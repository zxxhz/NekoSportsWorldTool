//! OBS 对象组装。
//!
//! 10 键对象，每值 gzip+base64；BD→GCJ 单次转换；27 键协议点集；
//! 10s 窗（speed/step_freq）；每 1000m 一圈；五点 fixed_point_json。
#![allow(non_snake_case)]

use chrono::{Local, TimeZone};
use flate2::write::GzEncoder;
use flate2::Compression;
use serde_json::{json, Value};
use std::io::Write;

use super::geom::round_to;
use super::model::{GenPoint, Track};

const X_PI: f64 = std::f64::consts::PI * 3000.0 / 180.0;

/// 百度 BD-09 → 高德 GCJ-02。
pub fn bd09_to_gcj02(bd_lat: f64, bd_lng: f64) -> (f64, f64) {
    let x = bd_lng - 0.0065;
    let y = bd_lat - 0.006;
    let z = (x * x + y * y).sqrt() - 0.00002 * (y * X_PI).sin();
    let theta = y.atan2(x) - 0.000003 * (x * X_PI).cos();
    (z * theta.sin(), z * theta.cos())
}

/// gzip + base64。
pub fn gz(data: &[u8]) -> String {
    let mut enc = GzEncoder::new(Vec::new(), Compression::default());
    enc.write_all(data).expect("gzip 写入内存失败");
    let out = enc.finish().expect("gzip 收尾失败");
    super::super::crypto::envelope::b64_encode(&out)
}

fn gz_json(v: &Value) -> String {
    gz(v.to_string().as_bytes())
}

fn gz_str(v: &str) -> String {
    gz(v.as_bytes())
}

/// 27 键协议点集（gen 点 → OBS 点；gLat/gLng 由 BD 转 GCJ，queueNum 映射对应 10s 分段序号）。
pub fn conv_point(p: &GenPoint, start_ms: i64) -> Value {
    let (glat, glng) = bd09_to_gcj02(p.gLat, p.gLng);
    let q_num = (p.totalTime / 10).max(0);
    json!({
        "avgSpeed": round_to(p.avgSpeed, 2),
        "bdA": round_to(p.bdA, 2),
        "bdD": round_to(p.bdD, 2),
        "bdG": p.bdG,
        "bdS": round_to(p.bdS, 2),
        "coorType": "gcj02",
        "count": p.count,
        "dtr": 0.0,
        "flag": start_ms,
        "gLat": round_to(glat, 7),
        "gLng": round_to(glng, 7),
        "gainTime": p.gainTime,
        "id": p.id,
        "lat": -1.0,
        "lng": -1.0,
        "locType": p.locType,
        "locationId": "",
        "queueNum": q_num,
        "radius": round_to(p.radius, 2),
        "speed": round_to(p.speed, 2),
        "state": p.state,
        "stepDistance": 0.0,
        "totalDis": round_to(p.totalDis, 4),
        "totalTime": p.totalTime,
        "type": p.ptype,
        "validDis": round_to(p.validDis, 4),
        "validTime": p.validTime,
    })
}

/// 判断轨迹是否实际经过某 BD-09 点位（任一轨迹点与打卡点距离 <= radius_m）。
pub fn track_passes_bd(track: &Track, bd_lat: f64, bd_lon: f64, radius_m: f64) -> bool {
    let r2 = radius_m * radius_m;
    track.locations.iter().any(|loc| {
        let dlat = (loc.gLat - bd_lat) * super::geom::MET_PER_DEG_LAT;
        let dlon = (loc.gLng - bd_lon) * super::geom::MET_PER_DEG_LNG;
        dlat * dlat + dlon * dlon <= r2
    })
}

/// 五点实体（实时点位 → 按轨迹实际是否经过计算 `isPass`）。
#[allow(dead_code)]
pub fn five_point_payload(points: &[Value], start_ms: i64) -> Vec<Value> {
    five_point_payload_for_track(points, None, start_ms)
}

/// 按轨迹实际落点距离（20m 内）判定每个打卡点的 `isPass` 状态。
pub fn five_point_payload_for_track(
    points: &[Value],
    track: Option<&Track>,
    start_ms: i64,
) -> Vec<Value> {
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let lat = p["lat"].as_f64().unwrap_or(0.0);
            let lon = p["lon"].as_f64().unwrap_or(0.0);
            let is_pass = match track {
                Some(t) => track_passes_bd(t, lat, lon, 20.0),
                None => true,
            };
            json!({
                "flag": start_ms,
                "glat": p["glat"].as_f64().unwrap_or(0.0),
                "glon": p["glon"].as_f64().unwrap_or(0.0),
                "id": i as i64 + 1,
                "isFixed": p["isFixed"].as_i64().unwrap_or(0),
                "isPass": is_pass,
                "lat": lat,
                "lon": lon,
                "pointName": p["pointName"].as_str().unwrap_or(""),
                "position": 999,
                "state": 0,
            })
        })
        .collect()
}

/// 提交 body 的 fivePointJson 包装串（默认全部通过）。
#[allow(dead_code)]
pub fn five_point_wrapper(points: &[Value], start_ms: i64) -> String {
    let five = five_point_payload(points, start_ms);
    json!({
        "useZip": false,
        "fivePointJson": Value::Array(five).to_string(),
        "runAreaId": -1,
        "geoFencesJson": "[]",
        "freedomShowFence": false,
    })
    .to_string()
}

/// 提交 body 的 fivePointJson 包装串（按轨迹实际经过的点位设置 `isPass`）。
pub fn five_point_wrapper_for_track(points: &[Value], track: &Track, start_ms: i64) -> String {
    let five = five_point_payload_for_track(points, Some(track), start_ms);
    json!({
        "useZip": false,
        "fivePointJson": Value::Array(five).to_string(),
        "runAreaId": -1,
        "geoFencesJson": "[]",
        "freedomShowFence": false,
    })
    .to_string()
}

/// 校验提交用五点轨迹仍是同一次点位请求产生的有效数据。
pub fn validate_five_point_wrapper(wrapper: &str) -> Result<(), String> {
    let outer: Value = serde_json::from_str(wrapper).map_err(|e| format!("五点轨迹 JSON 无效: {e}"))?;
    let raw = outer.get("fivePointJson").and_then(Value::as_str).ok_or("五点轨迹缺少 fivePointJson")?;
    let points: Vec<Value> = serde_json::from_str(raw).map_err(|e| format!("五点轨迹数组无效: {e}"))?;
    if points.is_empty() { return Err("五点轨迹不能为空".into()); }
    for point in points {
        let lat = point.get("lat").and_then(Value::as_f64).unwrap_or(0.0);
        let lon = point.get("lon").and_then(Value::as_f64).unwrap_or(0.0);
        let glat = point.get("glat").and_then(Value::as_f64).unwrap_or(0.0);
        let glon = point.get("glon").and_then(Value::as_f64).unwrap_or(0.0);
        if (lat == 0.0 && lon == 0.0) || (glat == 0.0 && glon == 0.0) { return Err("五点轨迹包含缺失坐标".into()); }
        crate::location::Coordinate::new(lat, lon, 0.0)?;
        crate::location::Coordinate::new(glat, glon, 0.0)?;
    }
    Ok(())
}

#[cfg(test)]
mod validation_tests {
    use super::*;
    #[test] fn rejects_empty_or_malformed_five_point_payload() { assert!(validate_five_point_wrapper("{}").is_err()); assert!(validate_five_point_wrapper(r#"{"fivePointJson":"[]"}"#).is_err()); }

    #[test]
    fn laps_are_rebuilt_from_overridden_altitude() {
        let points = vec![(38.901678, 121.540241), (38.902564, 121.541233)];
        let mut track = crate::track::generator::build(
            1200.0,
            600,
            7,
            (38.9, 121.54),
            1_700_000_000_000,
            &points,
        );
        crate::track::altitude::override_bd_a(&mut track, 36.75).unwrap();
        let laps = build_laps(&track, track.startTime);
        assert!(!laps.is_empty());
        assert!(laps.iter().all(|lap| lap["elevationGain"] == 0.0));
        assert!(laps.iter().all(|lap| lap["endAltAbs"] == 36.75));
        assert!(laps.iter().all(|lap| lap["endAltRel"] == 0.0));
    }

    #[test]
    fn five_points_only_mark_actually_visited_checkpoints_and_keep_pace_in_4_to_10_min() {
        let visited_pt = (38.901678, 121.540241);
        let next_pt = (38.906000, 121.540241);
        let unvisited_pt = (38.920000, 121.560000); // 距离路线 > 2km，不应被算作已打卡
        let wps = vec![visited_pt, next_pt];
        let track = crate::track::generator::build_ordered_mode(
            480.0,
            175,
            7,
            1_700_000_000_000,
            &wps,
            route_planner::CustomCloseMode::Open,
            &[visited_pt, next_pt, unvisited_pt],
        );

        // 1. 验证打卡点判定：路线经过的为 true，未经过的为 false
        let raw_points = vec![
            json!({ "lat": visited_pt.0, "lon": visited_pt.1, "glat": visited_pt.0, "glon": visited_pt.1, "pointName": "P1" }),
            json!({ "lat": unvisited_pt.0, "lon": unvisited_pt.1, "glat": unvisited_pt.0, "glon": unvisited_pt.1, "pointName": "P_FAR" }),
        ];
        let payload = five_point_payload_for_track(&raw_points, Some(&track), track.startTime);
        assert_eq!(payload[0]["isPass"], json!(true), "经过的打卡点应为 isPass=true");
        assert_eq!(payload[1]["isPass"], json!(false), "未经过的打卡点不应被算作 isPass=true");

        // 2. 验证配速严格保持在 4~10 分配速（1.66~4.17 m/s）内，无突然飙升
        for loc in track.locations.iter().skip(2) {
            if loc.ptype != -1 && loc.ptype != 6 && loc.avgSpeed > 0.0 {
                assert!(
                    loc.avgSpeed >= crate::track::generator::SPEED_FLOOR - 0.05
                        && loc.avgSpeed <= crate::track::generator::SPEED_CEIL + 0.05,
                    "瞬时速度 {} m/s 越出 4~10 分配速窗口",
                    loc.avgSpeed
                );
            }
        }

        // 3. 验证每个 10 秒时间窗等效位移率严格在 17.50~41.00 米之间，绝不下坠跌破 10'00"（绝不标红！）
        assert!(!track.speedPerTenSec.is_empty(), "10秒配速窗不能为空");
        for win in &track.speedPerTenSec {
            let equiv_10s = win.value / win.time as f64 * 10.0;
            assert!(
                equiv_10s >= 17.50 && equiv_10s <= 41.00,
                "10秒时间窗位移率 {} m 跌出 17.50~41.00m 安全区间（配速会标红）",
                equiv_10s
            );
        }
    }

    #[test]
    fn inspect_29min_run_monotonic_and_smooth() {
        let visited_pt = (38.901678, 121.540241);
        let next_pt = (38.906000, 121.540241);
        let wps = vec![visited_pt, next_pt];
        let track = crate::track::generator::build_ordered_mode(
            4125.0,
            1753,
            42,
            1_700_000_000_000,
            &wps,
            route_planner::CustomCloseMode::Open,
            &wps,
        );
        let base_pace_s = 1753.0 / (4125.0 / 1000.0); // 425.0s (7'05")

        // 验证 10 秒时间窗在基准配速自然生理波动走廊内生动起伏（如 7'05" 浮动在 6'35"~7'35" 内）
        assert!(!track.speedPerTenSec.is_empty());
        for win in &track.speedPerTenSec {
            let win_p = 1000.0 / (win.value / win.time as f64);
            assert!(
                (win_p - base_pace_s).abs() <= 29.5,
                "10秒窗配速 {:.1}s 越出目标基准配速 {:.1}s 自然波动走廊",
                win_p, base_pace_s
            );
        }

        // 验证 allLocJson 中的 speed 字段（定位 SDK 瞬时配速分/公里）处于自然生理波澜范围内
        for loc in &track.locations[2..track.locations.len() - 1] {
            let loc_pace_s = loc.speed * 60.0;
            assert!(
                (loc_pace_s - base_pace_s).abs() <= 31.0,
                "定位点配速 {:.1}s 越出目标配速 {:.1}s 自然波动范围",
                loc_pace_s, base_pace_s
            );
        }
        // 验证首点平滑等于真实起步点，哨兵 1 为全零点（对齐官方真机样本）
        assert_eq!(track.locations[0].speed, track.locations[2].speed);
        assert_eq!(track.locations[1].speed, 0.0);
    }

    #[test]
    fn test_user_real_track_for_pauses() {
        let cfg_path = "C:/Users/Emoing/Desktop/运动世界/ocy/config.json";
        if let Ok(content) = std::fs::read_to_string(cfg_path) {
            let v: serde_json::Value = serde_json::from_str(&content).unwrap();
            let wps_raw = v["custom_waypoints"].as_array().unwrap();
            let wps: Vec<(f64, f64)> = wps_raw
                .iter()
                .map(|p| (p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
                .collect();
            println!("Loaded {} custom waypoints", wps.len());
            let track = crate::track::generator::build_ordered_mode(
                4080.0,
                1780,
                42,
                1_700_000_000_000,
                &wps,
                route_planner::CustomCloseMode::Open,
                &wps,
            );
            println!("Generated track has {} locations", track.locations.len());
            let mut anomalous_speed = 0;
            let mut anomalous_state = 0;
            let mut anomalous_ptype = 0;
            let mut anomalous_bdg = 0;
            let mut zero_displacement = 0;

            for (i, loc) in track.locations.iter().enumerate() {
                if loc.state != 1 && loc.state != 2 && loc.state != 3 {
                    anomalous_state += 1;
                    println!("Point {} has state={}", i, loc.state);
                }
                if loc.bdG != 1 {
                    anomalous_bdg += 1;
                    println!("Point {} has bdG={}", i, loc.bdG);
                }
                if loc.ptype != 0 && loc.ptype != 5 && loc.ptype != 6 {
                    anomalous_ptype += 1;
                    println!("Point {} has ptype={}", i, loc.ptype);
                }
                if i > 1 && i < track.locations.len() - 1 {
                    let prev = &track.locations[i - 1];
                    let dlat = (loc.gLat - prev.gLat) * 111132.954;
                    let dlng = (loc.gLng - prev.gLng) * 97262.284;
                    let dist_step = (dlat * dlat + dlng * dlng).sqrt();
                    if dist_step < 0.2 {
                        zero_displacement += 1;
                        println!("Point {} zero displacement: {:.3}m, totalDis={:.2}", i, dist_step, loc.totalDis);
                    }
                }
                // Check conv_point representation
                let p_val = conv_point(loc, track.startTime);
                let speed_val = p_val["speed"].as_f64().unwrap();
                let _avg_sp_val = p_val["avgSpeed"].as_f64().unwrap();
                if speed_val <= 0.0 && i > 1 {
                    anomalous_speed += 1;
                    println!("Point {} conv_point speed={}", i, speed_val);
                }
            }
            assert_eq!(anomalous_state, 0, "不应存在异常状态点");
            assert_eq!(anomalous_bdg, 0, "不应存在丢星点");
            assert_eq!(anomalous_ptype, 0, "不应存在异常事件类型点");
            assert_eq!(zero_displacement, 0, "不应存在原地静止点");
            assert_eq!(anomalous_speed, 0, "不应存在零速点");

            // 验证 segment_json 完整构建且状态恒为 0 正常运动段
            let obj = build_obs_object(&track, 1340711571, "UUID-TEST", 16867125, &[]);
            let raw_seg = crate::crypto::envelope::b64_decode(obj["segment_json"].as_str().unwrap()).unwrap();
            let mut dec = flate2::read::GzDecoder::new(&raw_seg[..]);
            use std::io::Read;
            let mut s = String::new();
            dec.read_to_string(&mut s).unwrap();
            let segs_val: serde_json::Value = serde_json::from_str(&s).unwrap();
            assert_eq!(segs_val.as_array().unwrap().len(), track.speedPerTenSec.len());
            assert_eq!(segs_val[0]["state"], 0);
            println!("speedPerTenSec len: {}", track.speedPerTenSec.len());
            let mut slow_segments = 0;
            let mut fast_segments = 0;
            for i in 1..track.locations.len() {
                let p0 = &track.locations[i - 1];
                let p1 = &track.locations[i];
                let (g0_lat, g0_lng) = bd09_to_gcj02(p0.gLat, p0.gLng);
                let (g1_lat, g1_lng) = bd09_to_gcj02(p1.gLat, p1.gLng);
                let dlat = (g1_lat - g0_lat) * 111132.954;
                let dlng = (g1_lng - g0_lng) * 97262.284;
                let d = (dlat * dlat + dlng * dlng).sqrt();
                let dt = (p1.totalTime - p0.totalTime) as f64;
                let v = if dt > 0.0 { d / dt } else { 0.0 };
                if i > 2 && (v < 1.0 || d < 2.0) {
                    slow_segments += 1;
                    println!("Slow/stopped segment {}-{}: dist={:.2}m, dt={:.1}s, v={:.2}m/s", i - 1, i, d, dt, v);
                }
                if v > 6.0 {
                    fast_segments += 1;
                    println!("Fast segment {}-{}: dist={:.2}m, dt={:.1}s, v={:.2}m/s", i - 1, i, d, dt, v);
                }
            }
            println!("TOTAL slow_segments: {}, fast_segments: {}", slow_segments, fast_segments);
        }
    }
}

/// 10s 时间窗，id=(rrid%100000)*1000+窗口序秒（6 个真人样本跨 9 月记录验证一致；
/// 旧版 App 样本为全局序号，不适用当前版本）。
fn build_windows(track: &Track, rrid: i64) -> (Vec<Value>, Vec<Value>) {
    let start_ms = track.startTime;
    let total_time = track.totalTime;
    let mut sp = Vec::new();
    let mut stf = Vec::new();
    for (i, a) in track.speedPerTenSec.iter().enumerate() {
        let b = &track.stepsPerTenSec[i];
        let lo = (i * 10) as i64;
        let hi = (i * 10 + 10) as i64;
        let hi = hi.min(total_time);
        let id = (rrid % 100000) * 1000 + hi;
        sp.push(json!({
            "beginTime": start_ms + lo * 1000,
            "distance": a.value,
            "endTime": start_ms + hi * 1000,
            "flag": start_ms,
            "id": id,
            "queueNum": 0,
            "state": 0,
        }));
        stf.push(json!({
            "avgDiff": 0.0,
            "beginTime": start_ms + lo * 1000,
            "endTime": start_ms + hi * 1000,
            "flag": start_ms,
            "id": id,
            "maxDiff": 0.0,
            "minDiff": 1000.0,
            "queueNum": 0,
            "state": 0,
            "stepsNum": b.value as i64,
        }));
    }
    (sp, stf)
}

/// 圈（每 1000m 一圈，末圈 isFullLap=false；avgStride 单位厘米）。
fn build_laps(track: &Track, start_ms: i64) -> Vec<Value> {
    let mut laps = Vec::new();
    let locs = &track.locations;
    let (mut prev_d, mut prev_t, mut prev_steps, mut gain) = (0.0f64, 0i64, 0i64, 0.0f64);
    let alt0 = locs.first().map(|p| p.bdA).unwrap_or(0.0);
    for (i, pt) in locs.iter().enumerate() {
        if i > 0 {
            let dd = pt.bdA - locs[i - 1].bdA;
            if dd > 0.0 {
                gain += dd;
            }
        }
        let d_now = pt.totalDis;
        let t_now = pt.totalTime;
        let last = i == locs.len() - 1;
        if d_now - prev_d >= 1000.0 || last {
            let lap_d = d_now - prev_d;
            let lap_t = 1i64.max(t_now - prev_t);
            let lap_steps = pt.steps - prev_steps;
            laps.push(json!({
                "avgCadence": round_to(
                    (lap_steps as f64 / (lap_t as f64 / 60.0)).clamp(160.0, 210.0),
                    2,
                ),
                "avgPace": round_to(
                    ((lap_t as f64 / 60.0) / (lap_d / 1000.0).max(0.001)).clamp(4.05, 9.60),
                    2,
                ),
                "avgStride": round_to(lap_d / 1.max(lap_steps) as f64 * 100.0, 2),
                "cumulativeDuration": t_now,
                "distance": round_to(lap_d, 4),
                "duration": lap_t,
                "elevationGain": round_to(gain, 2),
                "endAltAbs": round_to(pt.bdA, 2),
                "endAltRel": round_to(pt.bdA - alt0, 2),
                "flag": start_ms,
                "id": laps.len() as i64 + 1,
                "isFullLap": lap_d >= 1000.0,
                "lapIndex": laps.len() as i64 + 1,
                "step": lap_steps,
            }));
            prev_d = d_now;
            prev_t = t_now;
            prev_steps = pt.steps;
            gain = 0.0;
        }
    }
    laps
}

/// 构造 10 秒分段 segment_json（对齐官方真机样本，每 10 秒一段，state 恒为 0 正常运动段）。
fn build_segments(track: &Track, start_ms: i64) -> Vec<Value> {
    let mut segs = Vec::new();
    let total_time = track.totalTime;
    for (i, win_sp) in track.speedPerTenSec.iter().enumerate() {
        let win_st = &track.stepsPerTenSec[i];
        let lo = (i * 10) as i64;
        let hi = ((i * 10 + win_sp.time as usize) as i64).min(total_time);
        let win_t = (hi - lo).max(1) as f64;
        let d = win_sp.value;
        let st = win_st.value;
        let pace_min = (win_t / 60.0) / (d / 1000.0).max(0.001);
        let cad_spm = (st / (win_t / 60.0)).round() as i64;
        segs.push(json!({
            "avgSpeed": pace_min,
            "avgStep": cad_spm,
            "distance": d,
            "endTime": start_ms + hi * 1000,
            "flag": start_ms,
            "id": i as i64 + 1,
            "intervalStepModel": 1,
            "originAvgStep": cad_spm,
            "speedStandard": 900,
            "startTime": start_ms + lo * 1000,
            "state": 0,
        }));
    }
    segs
}

/// 组装 10 键 OBS 对象（值均 gzip+base64；runFaceCheck 为空串 gzip）。
pub fn build_obs_object(
    track: &Track,
    rrid: i64,
    uuid: &str,
    uid: i64,
    live_points: &[Value],
) -> Value {
    let start_ms = track.startTime;
    let pts: Vec<Value> = track.locations.iter().map(|p| conv_point(p, start_ms)).collect();
    let run_wrap = json!({ "allLocJson": Value::Array(pts).to_string(), "useZip": false });
    let (sp, stf) = build_windows(track, rrid);
    let laps = build_laps(track, start_ms);
    let segs = build_segments(track, start_ms);
    let five = five_point_payload_for_track(live_points, Some(track), start_ms);
    let fx = json!({
        "fivePointJson": Value::Array(five).to_string(),
        "freedomShowFence": false,
        "geoFencesJson": "[]",
        "runAreaId": -1,
        "useZip": false,
    });
    json!({
        "rrid": gz_str(&rrid.to_string()),
        "uuid": gz_str(uuid),
        "uid": gz_str(&uid.to_string()),
        "run_data": gz_json(&run_wrap),
        "fixed_point_json": gz_json(&fx),
        "segment_json": gz_json(&Value::Array(segs)),
        "speed_json": gz_json(&Value::Array(sp)),
        "step_freq_json": gz_json(&Value::Array(stf)),
        "laps_json": gz_json(&Value::Array(laps)),
        "runFaceCheck": gz_str(""),
    })
}

/// OBS objectKey（两个都要传）。
pub fn obs_keys(track: &Track, rrid: i64, uuid: &str) -> Vec<String> {
    let t0 = Local
        .timestamp_millis_opt(track.startTime)
        .single()
        .map(|t| t.format("%Y%m%d%H").to_string())
        .unwrap_or_default();
    vec![
        format!("run_data/{t0}/{uuid}.json"),
        format!("run_data/{}/{}.json", rrid / 1000000, rrid),
    ]
}
