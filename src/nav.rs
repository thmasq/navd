use log::{info, trace, warn};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crate::state::{NavdContext, RobotState};
use crate::vision::AprilTagDetection;

const GOALPOST_SPACING_MM: f32 = 1000.0;
const T_LOST_MS: u64 = 1500;
const SENTINEL_TAG_ID: u16 = 587;
const YIELD_DISTANCE_MM: f32 = 500.0;
const BLEND_START_MM: f32 = 1000.0;

const KP_LAT: f32 = 0.05;
const KP_YAW: f32 = 20.0;
const BASE_SPEED: f32 = 40.0;
const APPROACH_SPEED: f32 = 20.0;

pub fn navigator_thread(ctx: &Arc<NavdContext>) {
    info!("Navigator thread started.");

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
            warn!("Lost visual contact for >{T_LOST_MS}ms. Transitioning to PANICKING.");
            ctx.state
                .store(RobotState::Panicking as u8, Ordering::Release);
            ctx.nav.update(0, 0);
            continue;
        }

        if let Some(snap) = snapshot {
            let now_us = crate::capture_timestamp_us();
            let frame_age_ms = now_us.saturating_sub(snap.timestamp_us) / 1000;

            if frame_age_ms > 100 {
                warn!("Vision data is stale ({frame_age_ms} ms old). Skipping frame.",);
                continue;
            }

            let active_tags = &snap.tags[0..snap.tag_count as usize];

            if current_state == RobotState::Navigating as u8
                && let Some(sentinel) = active_tags.iter().find(|t| t.id == SENTINEL_TAG_ID)
                && sentinel.distance_mm < YIELD_DISTANCE_MM
            {
                let distance = sentinel.distance_mm;
                info!("Reached sentinel tag ({distance}mm). Transitioning to YIELDING.",);
                ctx.state
                    .store(RobotState::Yielding as u8, Ordering::Release);
                ctx.nav.update(0, 0);
                continue;
            }

            let mut target_goalpost = ctx.nav.current_goalpost.load(Ordering::Relaxed);

            let mut left_tag = None;
            let mut right_tag = None;

            for tag in active_tags {
                if tag.id == target_goalpost {
                    left_tag = Some(tag);
                }
                if tag.id == target_goalpost + 1 {
                    right_tag = Some(tag);
                }
            }

            let crossed = match (left_tag, right_tag) {
                (Some(l), Some(r)) => l.tz < 0.0 || r.tz < 0.0,
                (Some(l), None) => l.tz < 0.0,
                (None, Some(r)) => r.tz < 0.0,
                (None, None) => false,
            };

            if crossed {
                target_goalpost += 2;
                ctx.nav
                    .current_goalpost
                    .store(target_goalpost, Ordering::Relaxed);
                info!(
                    "Crossed goalpost! Now tracking ({}, {})",
                    target_goalpost,
                    target_goalpost + 1
                );
            }

            if current_state == RobotState::Navigating as u8 {
                let (left_cmd, right_cmd) = calculate_steering(left_tag, right_tag);
                trace!("Calculated steering: L={}, R={}", left_cmd, right_cmd);
                ctx.nav.update(left_cmd, right_cmd);
            }
        }
    }
}

fn calculate_steering(
    left_tag: Option<&AprilTagDetection>,
    right_tag: Option<&AprilTagDetection>,
) -> (i8, i8) {
    if left_tag.is_none() && right_tag.is_none() {
        return (0, 0);
    }

    let closer_tag = match (left_tag, right_tag) {
        (Some(l), Some(r)) => {
            if l.distance_mm < r.distance_mm {
                l
            } else {
                r
            }
        }
        (Some(l), None) => l,
        (None, Some(r)) => r,
        _ => unreachable!(),
    };

    let current_base_speed = if closer_tag.distance_mm < 400.0 {
        APPROACH_SPEED
    } else {
        BASE_SPEED
    };

    let tx_target = match (left_tag, right_tag) {
        (Some(l), Some(r)) => f32::midpoint(l.tx, r.tx),
        (Some(l), None) => {
            let tx_visible = l.tx;
            let tx_inferred = tx_visible + GOALPOST_SPACING_MM;
            let tx_midpoint = f32::midpoint(tx_visible, tx_inferred);

            let w = (1.0 - (l.tz / BLEND_START_MM)).clamp(0.0, 0.6);
            tx_midpoint + w * (tx_visible - tx_midpoint)
        }
        (None, Some(r)) => {
            let tx_visible = r.tx;
            let tx_inferred = tx_visible - GOALPOST_SPACING_MM;
            let tx_midpoint = f32::midpoint(tx_visible, tx_inferred);

            let w = (1.0 - (r.tz / BLEND_START_MM)).clamp(0.0, 0.6);
            tx_midpoint + w * (tx_visible - tx_midpoint)
        }
        _ => 0.0,
    };

    let lateral_error = tx_target;
    let heading_error = closer_tag.yaw;

    let correction = KP_YAW.mul_add(heading_error, KP_LAT * lateral_error);

    let left_cmd = (current_base_speed + correction).clamp(-100.0, 100.0) as i8;
    let right_cmd = (current_base_speed - correction).clamp(-100.0, 100.0) as i8;

    (left_cmd, right_cmd)
}
