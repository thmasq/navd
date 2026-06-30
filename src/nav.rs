use log::{info, trace, warn};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crate::state::{NavdContext, RobotState};
use crate::vision::AprilTagDetection;

const T_LOST_MS: u64 = 1500;
const SENTINEL_TAG_ID: u16 = 587;
const YIELD_DISTANCE_MM: f32 = 500.0;

const KP_LAT: f32 = 0.05;
const KD_LAT: f32 = 0.02;
const D_FILTER_ALPHA: f32 = 0.2;

const BASE_SPEED: f32 = 40.0;
const APPROACH_SPEED: f32 = 20.0;

pub fn navigator_thread(ctx: &Arc<NavdContext>) {
    info!("Navigator thread started.");

    let unsafe_navigation = std::env::var("UNSAFE_NAVIGATION").is_ok();
    let mut logged_unsafe_lost = false;

    let mut last_tx_target = 0.0;
    let mut last_update_us = crate::capture_timestamp_us();
    let mut smoothed_d_tx = 0.0;

    loop {
        let snapshot = {
            let lock = ctx.vision.snapshot.lock().unwrap();
            let (new_lock, _timeout_result) = ctx
                .vision
                .new_frame_cv
                .wait_timeout(lock, Duration::from_millis(33))
                .unwrap();

            new_lock.clone()
        };

        let current_state = ctx.state.load(Ordering::Acquire);
        let now_ms = crate::capture_timestamp_us() / 1000;

        let last_seen = ctx.vision.last_tag_seen_ms.load(Ordering::Acquire);
        if current_state == RobotState::Navigating as u8
            && now_ms.saturating_sub(last_seen) > T_LOST_MS
        {
            if unsafe_navigation {
                if !logged_unsafe_lost {
                    warn!(
                        "Lost visual contact for >{T_LOST_MS}ms. UNSAFE_NAVIGATION is set, stopping motors and retrying."
                    );
                    logged_unsafe_lost = true;
                }
                ctx.nav.update(0, 0);
            } else {
                warn!("Lost visual contact for >{T_LOST_MS}ms. Transitioning to PANICKING.");
                ctx.state
                    .store(RobotState::Panicking as u8, Ordering::Release);
                ctx.nav.update(0, 0);
            }
            continue;
        } else {
            logged_unsafe_lost = false;
        }

        if let Some(snap) = snapshot {
            let now_us = crate::capture_timestamp_us();
            let frame_age_ms = now_us.saturating_sub(snap.timestamp_us) / 1000;

            if frame_age_ms > 400 {
                warn!("Vision data is stale ({frame_age_ms} ms old). Skipping frame.");
                continue;
            }

            let active_tags = &snap.tags[0..snap.tag_count as usize];

            if current_state == RobotState::Navigating as u8
                && let Some(sentinel) = active_tags.iter().find(|t| t.id == SENTINEL_TAG_ID)
                && sentinel.distance_mm < YIELD_DISTANCE_MM
            {
                let distance = sentinel.distance_mm;
                info!("Reached sentinel tag ({distance}mm). Transitioning to YIELDING.");
                ctx.state
                    .store(RobotState::Yielding as u8, Ordering::Release);
                ctx.nav.update(0, 0);
                continue;
            }

            let mut target_tag: Option<&AprilTagDetection> = None;
            for tag in active_tags {
                if tag.id != SENTINEL_TAG_ID {
                    if let Some(current) = target_tag {
                        if tag.id < current.id {
                            target_tag = Some(tag);
                        }
                    } else {
                        target_tag = Some(tag);
                    }
                }
            }

            if current_state == RobotState::Navigating as u8 {
                let (left_cmd, right_cmd) = calculate_steering(
                    target_tag,
                    &mut last_tx_target,
                    &mut last_update_us,
                    &mut smoothed_d_tx,
                );

                trace!("Calculated steering: L={}, R={}", left_cmd, right_cmd);
                ctx.nav.update(left_cmd, right_cmd);
            }
        }
    }
}

fn calculate_steering(
    target_tag: Option<&AprilTagDetection>,
    last_tx_target: &mut f32,
    last_update_us: &mut u64,
    smoothed_d_tx: &mut f32,
) -> (i8, i8) {
    let Some(tag) = target_tag else {
        return (0, 0);
    };

    let current_base_speed = if tag.distance_mm < 1800.0 {
        APPROACH_SPEED
    } else {
        BASE_SPEED
    };

    let tx_target = tag.tx;

    let now_us = crate::capture_timestamp_us();
    let dt_s = (now_us.saturating_sub(*last_update_us)) as f32 / 1_000_000.0;

    let raw_d_tx = if dt_s > 0.001 {
        (tx_target - *last_tx_target) / dt_s
    } else {
        0.0
    };

    *smoothed_d_tx = D_FILTER_ALPHA * raw_d_tx + (1.0 - D_FILTER_ALPHA) * (*smoothed_d_tx);

    *last_tx_target = tx_target;
    *last_update_us = now_us;

    let correction = KP_LAT * tx_target + KD_LAT * (*smoothed_d_tx);

    let left_cmd = (current_base_speed + correction).clamp(-100.0, 100.0) as i8;
    let right_cmd = (current_base_speed - correction).clamp(-100.0, 100.0) as i8;

    (left_cmd, right_cmd)
}
