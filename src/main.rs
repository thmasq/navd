#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap
)]

mod control;
mod nav;
mod sensor;
mod state;
mod uart;
mod udp;
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
        if let Ok(f) = OpenOptions::new().read(true).open("/dev/shm/tags.bin") {
            break f;
        }
        println!("Waiting for vision process to initialize /dev/shm/tags.bin...");
        std::thread::sleep(Duration::from_millis(500));
    };

    let mmap = unsafe {
        MmapOptions::new()
            .map(&file)
            .expect("Failed to mmap tags.bin")
    };

    let shared_tags_ptr = SharedMemoryPtr(mmap.as_ptr().cast::<vision::SharedTags>());

    let serial_port = serialport::new("/dev/ttyS0", 115_200)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open /dev/ttyS0");

    let port_for_control = serial_port
        .try_clone()
        .expect("Failed to clone serial port");
    let port_for_sensors = serial_port;

    // ---------------------------------------------------------
    // Thread 1: vision_reader (~30 Hz)
    // ---------------------------------------------------------
    let ctx_vision = Arc::clone(&ctx);

    std::thread::spawn(move || {
        let thread_safe_wrapper = shared_tags_ptr;

        loop {
            let shared = unsafe { &*thread_safe_wrapper.0 };

            if let Some(snapshot) = shared.read_seqlock() {
                let now_ms = capture_timestamp_us() / 1000;
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
        udp::listener_thread(&ctx_udp);
    });

    // ---------------------------------------------------------
    // Thread 3: navigator (~30 Hz)
    // ---------------------------------------------------------
    let ctx_nav = Arc::clone(&ctx);
    std::thread::spawn(move || {
        nav::navigator_thread(&ctx_nav);
    });

    // ---------------------------------------------------------
    // Thread 4: control (50 Hz)
    // ---------------------------------------------------------
    let ctx_control = Arc::clone(&ctx);
    std::thread::spawn(move || {
        control::control_thread(&ctx_control, port_for_control);
    });

    // ---------------------------------------------------------
    // Thread 5: sensor_poll (20 Hz)
    // ---------------------------------------------------------
    let ctx_sensors = Arc::clone(&ctx);
    std::thread::spawn(move || {
        sensor::sensor_poll_thread(&ctx_sensors, port_for_sensors);
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
