use serialport::SerialPort;
use std::io::Read;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crate::state::{NavdContext, RobotState};
use crate::uart;

pub fn sensor_poll_thread(ctx: Arc<NavdContext>, mut port: Box<dyn SerialPort>) {
    println!("Sensor poll thread started at 20 Hz.");

    let tick_duration = Duration::from_millis(50);

    loop {
        let start = Instant::now();

        let _ = port.clear(serialport::ClearBuffer::Input);

        let poll_frame = uart::build_frame(uart::CMD_SENSOR_POLL, &[]);
        if let Err(e) = port.write_all(&poll_frame) {
            eprintln!("UART Write Error (Sensor Poll): {}", e);
        }

        // Expected frame size: 1(AA) + 1(CMD) + 1(LEN) + 5(PAYLOAD) + 1(CRC) = 9 bytes
        let mut buf = vec![0u8; 32];
        let mut bytes_read = 0;

        for _ in 0..4 {
            match port.read(&mut buf[bytes_read..]) {
                Ok(n) if n > 0 => {
                    bytes_read += n;

                    if let Some(start_idx) = buf[..bytes_read]
                        .iter()
                        .position(|&b| b == uart::START_BYTE)
                    {
                        if bytes_read >= start_idx + 3 {
                            let cmd = buf[start_idx + 1];
                            let len = buf[start_idx + 2] as usize;
                            let total_frame_len = 3 + len + 1;

                            if bytes_read >= start_idx + total_frame_len {
                                let payload_start = start_idx + 3;
                                let payload_end = payload_start + len;
                                let crc_idx = payload_end;

                                let frame_for_crc = &buf[start_idx + 1..crc_idx];
                                let expected_crc = buf[crc_idx];

                                if uart::crc8_maxim(frame_for_crc) == expected_crc {
                                    if cmd == uart::CMD_SENSOR_STATUS && len == 5 {
                                        let flags = buf[payload_start];

                                        let mut heading_bytes = [0u8; 4];
                                        heading_bytes
                                            .copy_from_slice(&buf[payload_start + 1..payload_end]);
                                        let heading = f32::from_le_bytes(heading_bytes);

                                        ctx.sensors.update(flags, heading);

                                        if (flags & 0x07) != 0 {
                                            let state = ctx.state.load(Ordering::Acquire);
                                            if state == RobotState::Navigating as u8 {
                                                println!(
                                                    "CRITICAL: Collision detected! Transitioning to PANICKING."
                                                );
                                                ctx.state.store(
                                                    RobotState::Panicking as u8,
                                                    Ordering::Release,
                                                );
                                            }
                                        }
                                    }
                                } else {
                                    eprintln!("Sensor Poll: CRC mismatch");
                                }
                                break;
                            }
                        }
                    }
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => {
                    eprintln!("UART Read Error (Sensor Poll): {}", e);
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(4));
        }

        let elapsed = start.elapsed();
        if elapsed < tick_duration {
            std::thread::sleep(tick_duration - elapsed);
        }
    }
}
