use serde::Serialize;
use std::sync::atomic::{AtomicU32, Ordering};

pub const MAX_TAGS: usize = 20;

#[repr(C, packed)]
#[derive(Debug, Clone, Copy, Serialize)]
pub struct AprilTagDetection {
    pub id: u16,         // The decoded tag ID (0-586 for 36h11)
    pub hamming: u8,     // Number of bit errors corrected (0, 1, or 2)
    pub rotation: u8,    // Physical rotation in 90-deg increments (0-3)
    pub confidence: f32, // The decision margin from the GrayModel
    pub center_x: f32,   // Center pixel X
    pub center_y: f32,   // Center pixel Y

    pub tx: f32,
    pub ty: f32,
    pub tz: f32,

    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub distance_mm: f32,
}

#[repr(C)]
pub struct SharedTags {
    pub seq: AtomicU32,                      // offset 0
    pub tag_count: u32,                      // offset 4
    pub timestamp_us: u64,                   // offset 8
    pub frame_w: f32,                        // offset 16
    pub frame_h: f32,                        // offset 20
    pub pad: u64,                            // offset 24
    pub tags: [AprilTagDetection; MAX_TAGS], // offset 32
}

impl SharedTags {
    /// Safe seqlock read mechanism for Rust over shared memory
    pub fn read_seqlock(&self) -> Option<crate::state::VisionSnapshot> {
        loop {
            let seq1 = self.seq.load(Ordering::Acquire);

            // If writer is active (odd seq), spin
            if seq1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }

            let mut tags_copy = [unsafe { std::mem::zeroed::<AprilTagDetection>() }; MAX_TAGS];
            let tag_count_copy: u32;
            let ts_copy: u64;

            unsafe {
                std::ptr::copy_nonoverlapping(self.tags.as_ptr(), tags_copy.as_mut_ptr(), MAX_TAGS);
                tag_count_copy = std::ptr::read_volatile(&self.tag_count);
                ts_copy = std::ptr::read_volatile(&self.timestamp_us);
            }

            std::sync::atomic::compiler_fence(Ordering::Acquire);

            let seq2 = self.seq.load(Ordering::Acquire);

            if seq1 == seq2 {
                return Some(crate::state::VisionSnapshot {
                    timestamp_us: ts_copy,
                    tag_count: std::cmp::min(tag_count_copy, MAX_TAGS as u32),
                    tags: tags_copy,
                });
            }
        }
    }
}
