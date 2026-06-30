use log::{error, info, trace, warn};
use serialport::SerialPort;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crate::state::{NavdContext, RobotState};
use crate::uart;

pub fn control_thread(ctx: &Arc<NavdContext>, mut port: Box<dyn SerialPort>) {
    info!("Control thread started.");

    let record_file_path = std::env::var("RECORD").ok();
    let replay_file_path = std::env::var("REPLAY").ok();

    let override_mode = ctx.overrides.mode.load(Ordering::Acquire);

    let mut record_file = if override_mode == crate::state::OverrideMode::Record as u8 {
        if let Some(path) = record_file_path {
            info!("Record mode enabled. File: {}", path);
            Some(
                OpenOptions::new()
                    .create(true)
                    .write(true)
                    .truncate(true)
                    .open(path)
                    .expect("Failed to open record file"),
            )
        } else {
            None
        }
    } else {
        None
    };

    let replay_data = if override_mode == crate::state::OverrideMode::Replay as u8 {
        if let Some(path) = replay_file_path {
            info!("Replay mode enabled. File: {}", path);
            let data = std::fs::read_to_string(path).expect("Failed to read replay file");
            let mut entries = Vec::new();
            for line in data.lines() {
                let parts: Vec<&str> = line.split_whitespace().collect();
                if parts.len() >= 2 {
                    if let (Ok(dur), Ok(cmd)) = (parts[0].parse::<u64>(), parts[1].parse::<u8>()) {
                        let mut payload = Vec::new();
                        for p in &parts[2..] {
                            if let Ok(b) = p.parse::<u8>() {
                                payload.push(b);
                            }
                        }
                        entries.push((dur, cmd, payload));
                    }
                }
            }
            Some(entries)
        } else {
            None
        }
    } else {
        None
    };

    let mut current_rle: Option<(u64, u8, Vec<u8>)> = None;
    let mut replay_index = 0;
    let mut replay_entry_start_time = 0;

    loop {
        let state = ctx.state.load(Ordering::Acquire);
        let now_ms = crate::capture_timestamp_us() / 1000;

        let mut out_cmd = uart::CMD_STOP;
        let mut payload: Vec<u8> = vec![];
        let mut skip_normal_logic = false;

        if override_mode == crate::state::OverrideMode::Replay as u8 {
            if ctx.overrides.replay_trigger.swap(false, Ordering::AcqRel) {
                replay_index = 0;
                replay_entry_start_time = now_ms;
                info!("Replay started/restarted");
            }

            if let Some(ref entries) = replay_data {
                if replay_index < entries.len() {
                    let (dur, c, ref p) = entries[replay_index];
                    out_cmd = c;
                    payload = p.clone();

                    if now_ms.saturating_sub(replay_entry_start_time) >= dur {
                        replay_index += 1;
                        replay_entry_start_time += dur;
                    }
                    skip_normal_logic = true;
                } else {
                    out_cmd = uart::CMD_STOP;
                    payload.clear();
                    skip_normal_logic = true;
                }
            }
        }

        if !skip_normal_logic {
            if state == RobotState::Panicking as u8 {
                out_cmd = uart::CMD_ESTOP;
            } else if state == RobotState::RcOverride as u8 {
                let last_packet = ctx.rc.last_packet_ms.load(Ordering::Acquire);

                if now_ms.saturating_sub(last_packet) > 500 {
                    warn!("RC override timeout (>500ms). Stopping.");
                    out_cmd = uart::CMD_STOP;
                } else {
                    let rc = ctx.rc.read();

                    if (rc.flags & 0x01) != 0 {
                        out_cmd = uart::CMD_ESTOP;
                    } else {
                        // MUTUAL EXCLUSION: Lift vs Drive
                        let mut left = rc.left;
                        let mut right = rc.right;
                        let mut lift = rc.lift;

                        if left != 0 || right != 0 {
                            lift = 0;
                        } else if lift != 0 {
                            left = 0;
                            right = 0;
                        }

                        if lift != 0 {
                            trace!("Sending RC LIFT command: {}", lift);
                            out_cmd = uart::CMD_LIFT;
                            payload.push(lift as u8);
                        } else {
                            trace!("Sending RC DRIVE command: L:{}, R:{}", left, right);
                            out_cmd = uart::CMD_DRIVE;
                            payload.push(left as u8);
                            payload.push(right as u8);
                        }
                    }
                }
            } else if state == RobotState::Navigating as u8 {
                let nav = ctx.nav.read();
                out_cmd = uart::CMD_DRIVE;
                payload.push(nav.left as u8);
                payload.push(nav.right as u8);
            } else if state == RobotState::Yielding as u8 {
                out_cmd = uart::CMD_STOP;
            }
        }

        if override_mode == crate::state::OverrideMode::Record as u8 {
            let is_recording = ctx.overrides.is_recording.load(Ordering::Acquire);
            if is_recording {
                if let Some((start_time, c, ref p)) = current_rle {
                    if c != out_cmd || *p != payload {
                        let dur = now_ms.saturating_sub(start_time);
                        if let Some(ref mut f) = record_file {
                            let p_str: Vec<String> = p.iter().map(|b| b.to_string()).collect();
                            if let Err(e) = writeln!(f, "{} {} {}", dur, c, p_str.join(" ")) {
                                error!("Failed to write record: {e}");
                            }
                        }
                        current_rle = Some((now_ms, out_cmd, payload.clone()));
                    }
                } else {
                    info!("Recording started");
                    current_rle = Some((now_ms, out_cmd, payload.clone()));
                }
            } else {
                if let Some((start_time, c, ref p)) = current_rle {
                    let dur = now_ms.saturating_sub(start_time);
                    if let Some(ref mut f) = record_file {
                        let p_str: Vec<String> = p.iter().map(|b| b.to_string()).collect();
                        if let Err(e) = writeln!(f, "{} {} {}", dur, c, p_str.join(" ")) {
                            error!("Failed to write record: {e}");
                        }
                        let _ = f.flush();
                    }
                    info!("Recording stopped");
                    current_rle = None;
                }
            }
        }

        let frame = uart::build_frame(out_cmd, &payload);
        if let Err(e) = port.write_all(&frame) {
            error!("UART Write Error (Control): {e}");
        }

        std::thread::sleep(Duration::from_millis(20));
    }
}
