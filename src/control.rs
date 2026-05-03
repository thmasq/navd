use serialport::SerialPort;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crate::state::{NavdContext, RobotState};
use crate::uart;

pub fn control_thread(ctx: Arc<NavdContext>, mut port: Box<dyn SerialPort>) {
    println!("Control thread started at 50 Hz.");

    loop {
        let state = ctx.state.load(Ordering::Acquire);
        let now_ms = crate::capture_timestamp_us() / 1000;

        let mut out_cmd = uart::CMD_STOP;
        let mut payload: Vec<u8> = vec![];

        if state == RobotState::Panicking as u8 {
            out_cmd = uart::CMD_ESTOP;
        } else if state == RobotState::RcOverride as u8 {
            let last_packet = ctx.rc.last_packet_ms.load(Ordering::Acquire);

            if now_ms.saturating_sub(last_packet) > 500 {
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
                        lift = 0; // Drive takes precedence
                    } else if lift != 0 {
                        left = 0;
                        right = 0;
                    }

                    if lift != 0 {
                        out_cmd = uart::CMD_LIFT;
                        payload.push(lift as u8);
                    } else {
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

        let frame = uart::build_frame(out_cmd, &payload);
        if let Err(e) = port.write_all(&frame) {
            eprintln!("UART Write Error: {}", e);
        }

        std::thread::sleep(Duration::from_millis(20));
    }
}
