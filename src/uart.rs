pub const START_BYTE: u8 = 0xAA;

pub const CMD_DRIVE: u8 = 0x01;
pub const CMD_STOP: u8 = 0x02;
pub const CMD_ESTOP: u8 = 0x03;
pub const CMD_SENSOR_POLL: u8 = 0x04;
pub const CMD_LIFT: u8 = 0x05;
pub const CMD_SENSOR_STATUS: u8 = 0x10;

/// Calculates CRC-8/MAXIM (Dow-CRC)
/// Polynomial 0x31, Init 0x00, `RefIn` True, `RefOut` True, `XorOut` 0x00
pub fn crc8_maxim(data: &[u8]) -> u8 {
    let mut crc = 0x00;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

pub fn build_frame(cmd: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(4 + payload.len());
    frame.push(START_BYTE);
    frame.push(cmd);
    frame.push(payload.len() as u8);
    frame.extend_from_slice(payload);

    let crc = crc8_maxim(&frame[1..]);
    frame.push(crc);

    frame
}
