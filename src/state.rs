use crate::vision::MAX_TAGS;

use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, AtomicU16, AtomicU32, AtomicU64, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RobotState {
    Boot = 0,
    Navigating = 1,
    Yielding = 2,
    Panicking = 3,
    RcOverride = 4,
}

// ---------------
// Vision Domain
// ---------------
#[derive(Clone)]
pub struct VisionSnapshot {
    pub timestamp_us: u64,
    pub tag_count: u32,
    pub tags: [crate::vision::AprilTagDetection; MAX_TAGS],
}

pub struct VisionShared {
    pub snapshot: Mutex<Option<VisionSnapshot>>,
    pub new_frame_cv: std::sync::Condvar,
    pub last_tag_seen_ms: AtomicU64,
}

impl VisionShared {
    pub fn update(&self, new_data: VisionSnapshot) {
        let capture_time_ms = new_data.timestamp_us / 1000;

        if let Ok(mut snap) = self.snapshot.lock() {
            *snap = Some(new_data);
        }
        self.last_tag_seen_ms
            .store(capture_time_ms, Ordering::Release);
    }
}

// -------------------------
// Navigation & RC Domains
// -------------------------
pub struct NavCommand {
    pub left: i8,
    pub right: i8,
}

pub struct NavShared {
    cmd: AtomicU16,
    pub current_goalpost: std::sync::atomic::AtomicU16,
}

impl NavShared {
    pub fn update(&self, left: i8, right: i8) {
        let packed = (u16::from(left as u8) << 8) | u16::from(right as u8);
        self.cmd.store(packed, Ordering::Relaxed);
    }

    pub fn read(&self) -> NavCommand {
        let packed = self.cmd.load(Ordering::Relaxed);
        NavCommand {
            left: (packed >> 8) as i8,
            right: (packed & 0xFF) as i8,
        }
    }
}

pub struct RcCommand {
    pub left: i8,
    pub right: i8,
    pub lift: i8,
    pub flags: u8,
}

pub struct RcShared {
    cmd: AtomicU32,
    pub last_packet_ms: AtomicU64,
}

impl RcShared {
    pub fn update(&self, cmd: &RcCommand, time_ms: u64) {
        let packed = (u32::from(cmd.left as u8) << 24)
            | (u32::from(cmd.right as u8) << 16)
            | (u32::from(cmd.lift as u8) << 8)
            | u32::from(cmd.flags);

        self.cmd.store(packed, Ordering::Relaxed);
        self.last_packet_ms.store(time_ms, Ordering::Release);
    }

    pub fn read(&self) -> RcCommand {
        let packed = self.cmd.load(Ordering::Relaxed);
        RcCommand {
            left: (packed >> 24) as i8,
            right: ((packed >> 16) & 0xFF) as i8,
            lift: ((packed >> 8) & 0xFF) as i8,
            flags: (packed & 0xFF) as u8,
        }
    }
}

// ---------------
// Sensor Domain
// ---------------
pub struct SensorShared {
    pub flags: AtomicU8,
    pub heading_bits: AtomicU32,
}

impl SensorShared {
    pub fn update(&self, flags: u8, heading: f32) {
        self.heading_bits
            .store(heading.to_bits(), Ordering::Relaxed);
        self.flags.store(flags, Ordering::Release);
    }
}

// ----------------
// Global Context
// ----------------
pub struct NavdContext {
    pub state: AtomicU8,
    pub vision: VisionShared,
    pub nav: NavShared,
    pub rc: RcShared,
    pub sensors: SensorShared,
}

impl NavdContext {
    pub fn new() -> Self {
        let start_pair = if std::env::var("START_SMALLEST_PAIR").is_ok() {
            0
        } else if let Ok(val) = std::env::var("START_PAIR") {
            let mut p = val.parse::<u16>().unwrap_or(1);
            if p == 0 {
                p = 1;
            } else if p % 2 == 0 {
                p -= 1;
            }
            p
        } else {
            1
        };

        Self {
            state: AtomicU8::new(RobotState::Boot as u8),
            vision: VisionShared {
                snapshot: Mutex::new(None),
                new_frame_cv: std::sync::Condvar::new(),
                last_tag_seen_ms: AtomicU64::new(0),
            },
            nav: NavShared {
                cmd: AtomicU16::new(0),
                current_goalpost: std::sync::atomic::AtomicU16::new(start_pair),
            },
            rc: RcShared {
                cmd: AtomicU32::new(0),
                last_packet_ms: AtomicU64::new(0),
            },
            sensors: SensorShared {
                flags: AtomicU8::new(0),
                heading_bits: AtomicU32::new(0.0_f32.to_bits()),
            },
        }
    }
}
