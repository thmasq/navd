mod state;
mod vision;

use memmap2::MmapOptions;
use state::NavdContext;
use std::fs::OpenOptions;
use std::sync::Arc;
use std::time::Duration;

#[derive(Clone, Copy)]
struct SharedMemoryPtr(*const vision::SharedTags);
unsafe impl Send for SharedMemoryPtr {}

#[cfg(target_arch = "aarch64")]
static TIMER_FREQ: std::sync::OnceLock<u64> = std::sync::OnceLock::new();

fn main() {
    println!("Starting navd...");

    let ctx = Arc::new(NavdContext::new());

    let file = loop {
        match OpenOptions::new().read(true).open("/dev/shm/tags.bin") {
            Ok(f) => break f,
            Err(_) => {
                println!("Waiting for vision process to initialize /dev/shm/tags.bin...");
                std::thread::sleep(Duration::from_millis(500));
            }
        }
    };

    let mmap = unsafe {
        MmapOptions::new()
            .map(&file)
            .expect("Failed to mmap tags.bin")
    };

    let shared_tags_ptr = SharedMemoryPtr(mmap.as_ptr() as *const vision::SharedTags);

    // ---------------------------------------------------------
    // Thread 1: vision_reader (~30 Hz)
    // ---------------------------------------------------------
    let ctx_vision = Arc::clone(&ctx);

    std::thread::spawn(move || {
        let thread_safe_wrapper = shared_tags_ptr;

        loop {
            let shared = unsafe { &*thread_safe_wrapper.0 };

            if let Some(snapshot) = shared.read_seqlock() {
                let now_ms = capture_timestamp_us();
                ctx_vision.vision.update(snapshot, now_ms);
            }
            std::thread::sleep(Duration::from_millis(33)); // ~30 Hz
        }
    });

    // ---------------------------------------------------------
    // Thread 2: udp_listener (Event-driven)
    // ---------------------------------------------------------
    let ctx_udp = Arc::clone(&ctx);
    std::thread::spawn(move || {
        // let socket = UdpSocket::bind("0.0.0.0:5005").unwrap();
        loop {
            // Block on recvfrom
            // Decode 8-byte packet
            // ctx_udp.rc.update(cmd, now_ms);
            // Handle RC_OVERRIDE state transitions
            std::thread::sleep(Duration::from_millis(100)); // Stub
        }
    });

    // ---------------------------------------------------------
    // Thread 3: navigator (~30 Hz)
    // ---------------------------------------------------------
    let ctx_nav = Arc::clone(&ctx);
    std::thread::spawn(move || {
        loop {
            // Read ctx_nav.vision.snapshot
            // Run steering calc using Goalpost constraints
            // Update drive command
            // ctx_nav.nav.update(left_cmd, right_cmd);
            std::thread::sleep(Duration::from_millis(33));
        }
    });

    // ---------------------------------------------------------
    // Thread 4: control (50 Hz)
    // ---------------------------------------------------------
    let ctx_control = Arc::clone(&ctx);
    std::thread::spawn(move || {
        // Exclusive access to /dev/ttyAMA0 setup
        loop {
            // Check state
            // Read ctx_control.nav.read() OR ctx_control.rc.cmd.load()
            // Apply Lift Interlock guarantee
            // Build UART Frame and tx
            std::thread::sleep(Duration::from_millis(20)); // 50 Hz fixed tick
        }
    });

    // ---------------------------------------------------------
    // Thread 5: sensor_poll (20 Hz)
    // ---------------------------------------------------------
    let ctx_sensors = Arc::clone(&ctx);
    std::thread::spawn(move || {
        loop {
            // tx SENSOR_POLL over UART
            // rx SENSOR_STATUS
            // ctx_sensors.sensors.update(flags, heading);
            std::thread::sleep(Duration::from_millis(50)); // 20 Hz fixed tick
        }
    });

    loop {
        std::thread::park();
    }
}

#[cfg(target_arch = "aarch64")]
#[allow(clippy::inline_always)]
#[inline(always)]
pub fn capture_timestamp_us() -> u64 {
    let ticks: u64;

    unsafe {
        std::arch::asm!(
            "mrs {}, cntvct_el0",
            out(reg) ticks,
        );
    }

    let freq = *TIMER_FREQ.get_or_init(|| {
        let f: u64;
        unsafe {
            std::arch::asm!(
                "mrs {}, cntfrq_el0",
                out(reg) f,
            );
        }
        f
    });

    ((u128::from(ticks) * 1_000_000) / u128::from(freq)) as u64
}

#[cfg(not(target_arch = "aarch64"))]
pub fn capture_timestamp_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}
