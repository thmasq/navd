use log::{error, info, trace, warn};
use serialport::SerialPort;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crate::state::{NavdContext, RobotState};
use crate::uart;

struct Recorder {
    dir: std::path::PathBuf,
    stem: String,
    ext: String,
    counter: u32,
    file: Option<std::fs::File>,
    current_rle: Option<(u64, u8, Vec<u8>)>,
}

impl Recorder {
    fn new(path_str: &str) -> Self {
        let p = std::path::Path::new(path_str);
        let dir = p.parent().unwrap_or(std::path::Path::new("")).to_path_buf();
        let stem = p
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        let ext = p
            .extension()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        Self {
            dir,
            stem,
            ext,
            counter: 1,
            file: None,
            current_rle: None,
        }
    }

    fn update(&mut self, is_recording: bool, out_cmd: u8, payload: &[u8], now_ms: u64) {
        if is_recording {
            let needs_update = self
                .current_rle
                .as_ref()
                .map_or(false, |(_, c, p)| *c != out_cmd || p.as_slice() != payload);

            if needs_update {
                if let Some((start_time, c, p)) = self.current_rle.take() {
                    let dur = now_ms.saturating_sub(start_time);
                    self.write_entry(dur, c, &p);
                }
                self.current_rle = Some((now_ms, out_cmd, payload.to_vec()));
            } else if self.current_rle.is_none() {
                self.open_next_file();
                self.current_rle = Some((now_ms, out_cmd, payload.to_vec()));
            }
        } else {
            if let Some((start_time, c, p)) = self.current_rle.take() {
                let dur = now_ms.saturating_sub(start_time);
                self.write_entry(dur, c, &p);
                if let Some(ref mut f) = self.file {
                    let _ = f.flush();
                }
                info!("Recording stopped");
                self.file = None;
            }
        }
    }

    fn write_entry(&mut self, dur: u64, cmd: u8, payload: &[u8]) {
        if let Some(ref mut f) = self.file {
            let p_str: Vec<String> = payload.iter().map(|b| b.to_string()).collect();
            if let Err(e) = writeln!(f, "{} {} {}", dur, cmd, p_str.join(" ")) {
                error!("Failed to write record: {e}");
            }
        }
    }

    fn open_next_file(&mut self) {
        loop {
            let mut filename = format!("{}-{}", self.stem, self.counter);
            if !self.ext.is_empty() {
                filename.push('.');
                filename.push_str(&self.ext);
            }
            let full_path = self.dir.join(&filename);
            if !full_path.exists() {
                info!("Recording started. File: {:?}", full_path);
                self.file = OpenOptions::new()
                    .create(true)
                    .write(true)
                    .truncate(true)
                    .open(&full_path)
                    .ok();
                if self.file.is_none() {
                    error!("Failed to open record file {:?}", full_path);
                }
                break;
            }
            self.counter += 1;
        }
    }
}

struct Replayer {
    entries: Vec<(u64, u8, Vec<u8>)>,
    index: usize,
    entry_start_time: u64,
}

impl Replayer {
    fn new(path: &str) -> Option<Self> {
        let data = std::fs::read_to_string(path).ok()?;
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
        info!("Replay mode enabled. File: {}", path);
        Some(Self {
            entries,
            index: usize::MAX,
            entry_start_time: 0,
        })
    }

    fn trigger(&mut self, now_ms: u64) {
        self.index = 0;
        self.entry_start_time = now_ms;
        info!("Replay started/restarted");
    }

    fn abort(&mut self) {
        if self.is_active() {
            warn!("RC Override detected! Aborting replay.");
            self.index = usize::MAX;
        }
    }

    fn is_active(&self) -> bool {
        self.index < self.entries.len()
    }

    fn get_command(&mut self, now_ms: u64) -> (u8, Vec<u8>) {
        while self.index < self.entries.len() {
            let dur = self.entries[self.index].0;
            if now_ms.saturating_sub(self.entry_start_time) >= dur {
                self.index += 1;
                self.entry_start_time += dur;
            } else {
                break;
            }
        }

        if self.index < self.entries.len() {
            let (_, c, ref p) = self.entries[self.index];
            (c, p.clone())
        } else {
            (uart::CMD_STOP, vec![])
        }
    }
}

fn get_normal_command(ctx: &Arc<NavdContext>, state: u8, now_ms: u64) -> (u8, Vec<u8>) {
    let mut out_cmd = uart::CMD_STOP;
    let mut payload = Vec::new();

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

    (out_cmd, payload)
}

pub fn control_thread(ctx: &Arc<NavdContext>, mut port: Box<dyn SerialPort>) {
    info!("Control thread started.");

    let override_mode = ctx.overrides.mode.load(Ordering::Acquire);

    let mut recorder = if override_mode == crate::state::OverrideMode::Record as u8 {
        std::env::var("RECORD").ok().map(|p| Recorder::new(&p))
    } else {
        None
    };

    let mut replayer = if override_mode == crate::state::OverrideMode::Replay as u8 {
        std::env::var("REPLAY").ok().and_then(|p| Replayer::new(&p))
    } else {
        None
    };

    loop {
        let state = ctx.state.load(Ordering::Acquire);
        let now_ms = crate::capture_timestamp_us() / 1000;

        let mut out_cmd = uart::CMD_STOP;
        let mut payload = vec![];
        let mut skip_normal_logic = false;

        if override_mode == crate::state::OverrideMode::Replay as u8 {
            if let Some(ref mut rep) = replayer {
                if ctx.overrides.replay_trigger.swap(false, Ordering::AcqRel) {
                    rep.trigger(now_ms);
                }

                if state == RobotState::RcOverride as u8 && rep.is_active() {
                    rep.abort();
                }

                if rep.is_active() {
                    let (c, p) = rep.get_command(now_ms);
                    out_cmd = c;
                    payload = p;
                    skip_normal_logic = true;
                }
            }
        }

        if !skip_normal_logic {
            let (c, p) = get_normal_command(ctx, state, now_ms);
            out_cmd = c;
            payload = p;
        }

        if override_mode == crate::state::OverrideMode::Record as u8 {
            let is_recording = ctx.overrides.is_recording.load(Ordering::Acquire);
            if let Some(ref mut rec) = recorder {
                rec.update(is_recording, out_cmd, &payload, now_ms);
            }
        }

        let frame = uart::build_frame(out_cmd, &payload);
        if let Err(e) = port.write_all(&frame) {
            error!("UART Write Error (Control): {e}");
        }

        std::thread::sleep(Duration::from_millis(20));
    }
}
