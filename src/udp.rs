#![allow(clippy::similar_names)]

use log::{error, info, warn};
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::Ordering;

use crate::state::{NavdContext, RcCommand, RobotState};

pub fn listener_thread(ctx: &Arc<NavdContext>) {
    let socket = UdpSocket::bind("0.0.0.0:5005").expect("Failed to bind UDP port");

    socket
        .set_read_timeout(Some(std::time::Duration::from_millis(500)))
        .unwrap();

    let mut buf = [0u8; 8];

    info!("UDP Listener bound to port 5005");

    loop {
        match socket.recv_from(&mut buf) {
            Ok((size, _src)) => {
                if size == 8 {
                    handle_packet(ctx, buf);
                } else {
                    warn!("Received malformed UDP packet of size {size}");
                }
            }
            Err(e) => {
                if e.kind() != std::io::ErrorKind::WouldBlock {
                    error!("UDP Socket error: {e}");
                }
            }
        }
    }
}

fn handle_packet(ctx: &Arc<NavdContext>, buf: [u8; 8]) {
    let packet_type = buf[0];
    let left = buf[1] as i8;
    let right = buf[2] as i8;
    let flags = buf[3];
    let lift = buf[4] as i8;
    // buf[5..8] are reserved

    let now_ms = crate::capture_timestamp_us() / 1000;

    match packet_type {
        0x01 => {
            // TYPE = 0x01: Drive/Operate command
            let current_state = ctx.state.load(Ordering::Acquire);

            // Edge-trigger transition to RC_OVERRIDE if not already in it
            if current_state != RobotState::RcOverride as u8 {
                ctx.state
                    .store(RobotState::RcOverride as u8, Ordering::Release);
                info!("Transitioned to RC_OVERRIDE via UDP command");
            }

            let cmd = RcCommand {
                left,
                right,
                lift,
                flags,
            };
            ctx.rc.update(&cmd, now_ms);
        }
        0x02 => {
            let override_mode = ctx.overrides.mode.load(Ordering::Acquire);

            if override_mode == crate::state::OverrideMode::Record as u8 {
                let current = ctx.overrides.is_recording.load(Ordering::Acquire);
                ctx.overrides
                    .is_recording
                    .store(!current, Ordering::Release);
                info!("Recording toggled: {}", !current);
            } else if override_mode == crate::state::OverrideMode::Replay as u8 {
                ctx.overrides.replay_trigger.store(true, Ordering::Release);
                info!("Replay triggered from start");
            } else {
                let current_state = ctx.state.load(Ordering::Acquire);

                if current_state == RobotState::RcOverride as u8 {
                    let is_visible = ctx.vision.snapshot.lock().is_ok_and(|snap_lock| {
                        snap_lock.as_ref().is_some_and(|snap| snap.tag_count > 0)
                    });

                    if is_visible {
                        info!("Tags are visible. Resuming NAVIGATING.");
                        ctx.state
                            .store(RobotState::Navigating as u8, Ordering::Release);
                    } else {
                        warn!("Command rejected: No tags are visible. Staying in RC.");
                    }
                }
            }
        }
        _ => {
            warn!("Received unknown UDP packet TYPE: {packet_type:#04X}");
        }
    }
}
