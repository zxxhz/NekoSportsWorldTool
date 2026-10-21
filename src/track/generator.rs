//! 自然轨迹生成器。
//!
//! 画像驱动：打卡点拟合闭合环（Catmull-Rom，每段 18 采样）→ 弧长表；
//! 采样间隔主 5s（80%）；速度曲线 = ramp × 疲劳 × 三正弦 × 余弦凹陷 × 微噪；
//! 正常点按速度曲线分配位移并归一到精确总距离；异常点（-1）零位移/跳变；
//! 哨兵点（首 type∈{0,7}、索引1 type=5、末 type=6）；结尾断崖；点位吸附。
#![allow(non_snake_case)]

use super::geom::{
    fmt_gain_time, make_ordered_path, make_point_ring, open_point_at, ring_point_at, round_to,
    to_bd, Rng, MET_PER_DEG_LAT, MET_PER_DEG_LNG,
};
use super::model::{GenPoint, Segment, TenWindow, Track};
use super::postfix::apply_post_fixes;

/// 有效配速窗口：4'00"-10'00"/km（即 240-600 秒/km ≈ 1.667-4.167 m/s），留安全余量（约 4'04"-9'41"/km）。
pub const SPEED_FLOOR: f64 = 1.72;
pub const SPEED_CEIL: f64 = 4.10;

/// 等比缩放逐点速度至目标总距：越界点钳在 `[floor, ceil]` 边界，剩余差量由未饱和点分摊（迭代收敛）。
pub fn fit_speeds_bounded(w: &mut [f64], dts: &[f64], target: f64, floor: f64, ceil: f64) {
    let floor = floor.clamp(SPEED_FLOOR, SPEED_CEIL);
    let ceil = ceil.clamp(floor, SPEED_CEIL);
    for _ in 0..32 {
        let cur: f64 = w.iter().zip(dts).map(|(x, dt)| x * dt).sum();
        if (cur - target).abs() <= 0.5 || cur <= 1e-9 {
            break;
        }
        let k = target / cur;
        for x in w.iter_mut() {
            if *x > 0.0 {
                *x = (*x * k).clamp(floor, ceil);
            }
        }
    }
    for x in w.iter_mut() {
        if *x > 0.0 {
            *x = x.clamp(floor, ceil);
        }
    }
}

/// 平滑 10 秒时间窗（围绕目标配速基准线自然起伏波动 ±28 秒左右，展现逼真的人类跑步生理波澜，如 6'20" 自然浮动在 5'52"~6'48"）。
pub fn smooth_ten_sec_windows(
    dur: i64,
    dist: f64,
    seed: u64,
    allow_mid_slowdown: bool,
    windows_speed: &mut Vec<TenWindow>,
    windows_steps: &mut Vec<TenWindow>,
) {
    if windows_speed.is_empty() {
        return;
    }
    let m = windows_speed.len();
    let dur_f = dur as f64;
    let base_v = (dist / dur_f.max(1.0)).clamp(SPEED_FLOOR + 0.05, SPEED_CEIL - 0.05);
    let base_pace_s = 1000.0 / base_v;
    let mut rng = Rng::new(seed.wrapping_add(0x85EB_CA6B_14E5_B231));

    let phi1 = ((seed & 0xFFFF) as f64 / 65535.0) * std::f64::consts::TAU;
    let phi2 = (((seed >> 16) & 0xFFFF) as f64 / 65535.0) * std::f64::consts::TAU;

    let mut speeds: Vec<f64> = Vec::with_capacity(m);
    let mut walk_dips: Vec<f64> = vec![0.0; m];
    let mut drift_s = 0.0f64;

    for i in 0..m {
        let win_t = windows_speed[i].time as f64;
        let mid_t = (i as f64 * 10.0 + win_t * 0.5).min(dur_f);
        let u = (mid_t / dur_f.max(1.0)).clamp(0.0, 1.0);
        // 体能节律趋势：前期微快（约 -8s），后程疲劳轻微放缓（约 +8s）
        let s_curve = 3.0 * u * u - 2.0 * u * u * u;
        let trend_s = -8.0 + 16.0 * s_curve;
        // 多尺度自然生理律动波：呼吸与肌肉收缩宏观波澜（幅度约 ±18s）
        let wave_s = 10.0 * (std::f64::consts::TAU * mid_t / 140.0 + phi1).sin()
            + 6.5 * (std::f64::consts::TAU * mid_t / 75.0 + phi2).cos()
            + 4.0 * (std::f64::consts::TAU * mid_t / 35.0 + phi1 * 2.0).sin();
        drift_s = 0.80 * drift_s + rng.gauss(0.0, 0.85);
        let mut delta_s = (trend_s + wave_s + drift_s).clamp(-28.0, 28.0);

        // 若开启中途走步歇息设置：在行程 52%~64% 处（约中段）插入一次平滑自然的走步休整（配速放缓至约 8'10"，处于合规运动窗口内，杜绝触发暂停）
        if allow_mid_slowdown && m >= 8 {
            let u_mid = (u - 0.58) / 0.055;
            if u_mid.abs() < 2.5 {
                let dip = (-0.5 * u_mid * u_mid).exp();
                walk_dips[i] = dip;
                let walk_pace_s = 490.0; // 约 8'10" 走步慢行（严格限制在高校 8'30"~9'00" 合规限内）
                let target_p = base_pace_s + delta_s;
                let blended_p = target_p * (1.0 - 0.90 * dip) + walk_pace_s * (0.90 * dip);
                delta_s = blended_p - base_pace_s;
            }
        }

        let cur_pace_s = (base_pace_s + delta_s).clamp(240.0, 600.0);
        let cur_v = (1000.0 / cur_pace_s).clamp(SPEED_FLOOR, SPEED_CEIL);
        speeds.push(cur_v);
    }

    // 相邻窗口平滑限斜率（slew rate limiter）：容许自然起伏的同时杜绝生硬台阶
    for i in 1..m {
        let max_dv = 0.09;
        let diff = speeds[i] - speeds[i - 1];
        if diff.abs() > max_dv {
            speeds[i] = speeds[i - 1] + diff.signum() * max_dv;
        }
    }
    // 逆向反扫一次，保证双向连续无突兀拐点
    for i in (0..m - 1).rev() {
        let max_dv = 0.09;
        let diff = speeds[i] - speeds[i + 1];
        if diff.abs() > max_dv {
            speeds[i] = speeds[i + 1] + diff.signum() * max_dv;
        }
    }

    // 由连续速度换算为窗口位移 vals[i] = speeds[i] * win_t
    let mut vals: Vec<f64> = (0..m)
        .map(|i| speeds[i] * (windows_speed[i].time as f64))
        .collect();

    // 归一化缩放：精确守恒目标总位移
    let cur_sum: f64 = vals.iter().sum();
    if cur_sum > 1e-3 {
        let k = dist / cur_sum;
        for (i, v) in vals.iter_mut().enumerate() {
            let win_t = windows_speed[i].time as f64;
            let floor_d = 17.50 * (win_t / 10.0);
            let ceil_d = 41.00 * (win_t / 10.0);
            *v = (*v * k).clamp(floor_d, ceil_d);
        }
    }

    // 联动计算步频步数：一开始步频慢（前 45 秒从 ~160-168 spm 平滑爬升至均值），
    // 达到均值（~183 spm）后在 168~196 spm 之间生动自然起伏波动
    let mut steps_vals: Vec<f64> = Vec::with_capacity(m);
    for i in 0..m {
        let win_t = windows_speed[i].time as f64;
        let mid_t = (i as f64 * 10.0 + win_t * 0.5).min(dur_f);
        let d = vals[i];
        let v = d / win_t.max(0.1);

        // 起步热身：前 45 秒从较慢步频平滑爬升至巡航均值
        let warmup_u = (mid_t / 45.0).clamp(0.0, 1.0);
        let warmup_s = 3.0 * warmup_u * warmup_u - 2.0 * warmup_u * warmup_u * warmup_u;
        let warmup_delta = -20.0 * (1.0 - warmup_s);

        let cad_wave = 5.0 * (std::f64::consts::TAU * mid_t / 170.0 + phi1).sin()
            + 3.5 * (std::f64::consts::TAU * mid_t / 95.0 + phi2).cos();
        let run_cad = (183.0 + warmup_delta + 2.5 * (v - base_v) + cad_wave + rng.gauss(0.0, 3.2)).clamp(152.0, 198.0);
        let walk_cad = (118.0 + rng.gauss(0.0, 3.0)).clamp(108.0, 128.0);
        let dip = walk_dips[i];
        let cad = if dip > 0.05 {
            run_cad * (1.0 - 0.85 * dip) + walk_cad * (0.85 * dip)
        } else {
            run_cad
        };
        let st = (cad / 60.0 * win_t).round().max(1.0);
        steps_vals.push(st.clamp((win_t * 1.6).round().max(1.0), (win_t * 4.2).round().max(1.0)));
    }

    // 回写到原数组
    for i in 0..m {
        windows_speed[i].value = round_to(vals[i], 2);
        windows_steps[i].value = round_to(steps_vals[i], 0);
    }
}

/// 轨迹生成主入口。points_bd 为 BD 系打卡点。
pub fn build(
    dist: f64,
    dur: i64,
    seed: u64,
    _center: (f64, f64),
    start_ms: i64,
    points_bd: &[(f64, f64)],
) -> Track {
    let (dense, arcs, pc) = make_point_ring(points_bd);
    build_from_ring(
        dist, dur, seed, start_ms, &dense, &arcs, pc, false, true, points_bd,
    )
}

/// 无路网自定义途经点轨迹生成（兼容布尔闭合标志）。
#[allow(dead_code)]
pub fn build_ordered(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    ordered_bd: &[(f64, f64)],
    close_loop: bool,
    snap_bd: &[(f64, f64)],
) -> Track {
    let mode = if close_loop {
        route_planner::CustomCloseMode::Loop
    } else {
        route_planner::CustomCloseMode::OutAndBack
    };
    build_ordered_mode(dist, dur, seed, start_ms, ordered_bd, mode, snap_bd)
}

/// 按自定义闭合模式（含 `Open` 开环单程、`Loop` 闭合环线、`OutAndBack` 原路折返）生成轨迹。
pub fn build_ordered_mode(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    ordered_bd: &[(f64, f64)],
    mode: route_planner::CustomCloseMode,
    snap_bd: &[(f64, f64)],
) -> Track {
    let (dense, arcs, pc) = make_ordered_path(ordered_bd, mode);
    build_from_ring(
        dist,
        dur,
        seed,
        start_ms,
        &dense,
        &arcs,
        pc,
        true,
        mode.is_closed(),
        snap_bd,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_from_ring(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    dense: &[(f64, f64)],
    arcs: &[f64],
    pc: (f64, f64),
    pin_start: bool,
    closed: bool,
    points_bd: &[(f64, f64)],
) -> Track {
    let mut rng = Rng::new(seed);
    let dur_f = dur as f64;
    let (c_lat, c_lng) = pc;
    let direction: f64 = if pin_start {
        1.0
    } else {
        rng.choice(&[1.0, -1.0])
    };
    let s0 = if pin_start {
        0.0
    } else {
        rng.uniform(0.0, *arcs.last().unwrap_or(&400.0))
    };
    let _phase_v = rng.uniform(0.0, std::f64::consts::TAU);
    let _phase_l = rng.uniform(0.0, std::f64::consts::TAU);

    let mut times = Vec::new();
    let mut t = 0.0;
    while t < dur_f {
        times.push(t);
        t += if rng.random() < 0.80 {
            5.0
        } else {
            rng.choice(&[3.0, 4.0, 5.0, 5.0, 6.0, 7.0])
        };
    }
    let n = times.len();
    let mut dts: Vec<f64> = (0..n - 1).map(|i| times[i + 1] - times[i]).collect();
    dts.push(1.0f64.max(dur_f - times[n - 1]));

    let base = (dist / dur_f).clamp(SPEED_FLOOR + 0.05, SPEED_CEIL - 0.05);
    let allow_mid_slowdown = crate::api::model::load_config().allow_mid_slowdown;
    let local_floor = if allow_mid_slowdown {
        SPEED_FLOOR
    } else {
        (base * 0.80).max(SPEED_FLOOR)
    };
    let local_ceil = (base * 1.20).min(SPEED_CEIL).max(local_floor + 0.05);

    // 围绕目标配速基准线自然起伏波动 ±28 秒左右，还原真实学生的跑步波动
    let base_pace_s = 1000.0 / base.max(0.1);
    let phi1 = ((seed & 0xFFFF) as f64 / 65535.0) * std::f64::consts::TAU;
    let phi2 = (((seed >> 16) & 0xFFFF) as f64 / 65535.0) * std::f64::consts::TAU;

    let mut w = Vec::with_capacity(n);
    let mut drift_s = 0.0f64;
    for i in 0..n {
        let tt = times[i];
        let _dt = dts[i];
        let u = (tt / dur_f).clamp(0.0, 1.0);
        let s_curve = 3.0 * u * u - 2.0 * u * u * u;
        let trend_s = -8.0 + 16.0 * s_curve;
        let wave_s = 10.0 * (std::f64::consts::TAU * tt / 140.0 + phi1).sin()
            + 6.5 * (std::f64::consts::TAU * tt / 75.0 + phi2).cos()
            + 4.0 * (std::f64::consts::TAU * tt / 35.0 + phi1 * 2.0).sin();
        drift_s = 0.80 * drift_s + rng.gauss(0.0, 0.85);
        let mut delta_s = (trend_s + wave_s + drift_s).clamp(-28.0, 28.0);

        // 若开启中途走步歇息：在行程 52%~64% 处平滑插入走步放慢（降至约 8'10"，处于合规运动窗口内，杜绝触发暂停）
        if allow_mid_slowdown && n >= 20 {
            let u_mid = (u - 0.58) / 0.055;
            if u_mid.abs() < 2.5 {
                let dip = (-0.5 * u_mid * u_mid).exp();
                let walk_pace_s = 490.0;
                let target_p = base_pace_s + delta_s;
                let blended_p = target_p * (1.0 - 0.90 * dip) + walk_pace_s * (0.90 * dip);
                delta_s = blended_p - base_pace_s;
            }
        }

        let cur_pace_s = (base_pace_s + delta_s).clamp(240.0, 600.0);
        let cur_v = (1000.0 / cur_pace_s).clamp(SPEED_FLOOR, SPEED_CEIL);
        w.push(cur_v);
    }

    let mut kinds: Vec<(i64, i64)> = Vec::with_capacity(n);
    for _ in 0..n {
        let u = rng.random();
        // 全程纯绿有效定位点：type=0（62%）与 type=3（38%），绝不注入 type=1/2（暂停）或 type=-1（断点）
        if u < 0.38 {
            kinds.push((3, 1));
        } else {
            kinds.push((0, 1));
        }
    }

    // 仅哨兵点 i<=1 不计位移，所有跑步过程中的正常点均连续向前推进
    for i in 0..n {
        if i <= 1 {
            w[i] = 0.0;
        }
    }
    fit_speeds_bounded(&mut w, &dts, dist, local_floor, local_ceil);
    // 若温和窗口 [local_floor, local_ceil] 已饱和仍差少量距离，放宽到 [SPEED_FLOOR, SPEED_CEIL] 收敛
    let cur_sum: f64 = w.iter().zip(&dts).map(|(x, dt)| x * dt).sum();
    if (cur_sum - dist).abs() > 2.0 {
        fit_speeds_bounded(&mut w, &dts, dist, SPEED_FLOOR, SPEED_CEIL);
    }
    let disp_of: Vec<f64> = (0..n).map(|i| w[i] * dts[i]).collect();
    let seg_dist: Vec<f64> = disp_of.clone();
    let speeds: Vec<f64> = (0..n)
        .map(|i| if w[i] > 0.0 { w[i] } else { base })
        .collect();

    let mut locs: Vec<GenPoint> = Vec::with_capacity(n);
    let mut s = s0;
    let mut t_acc = 0.0f64;
    let mut dist_acc = 0.0f64;
    let mut steps_acc = 0.0f64;
    let mut jx = 0.0f64;
    let mut jy = 0.0f64;
    let mut alt = 82.0 + rng.uniform(-1.0, 1.0);
    let n_est = 1.max((dur_f / 5.0) as i64);
    let alt_sigma = rng.uniform(3.8, 6.2) / (0.40 * n_est as f64);
    let mut ten_t = 0.0f64;
    let mut ten_d = 0.0f64;
    let mut ten_st = 0.0f64;
    let mut ten_speed: Vec<TenWindow> = Vec::new();
    let mut ten_steps: Vec<TenWindow> = Vec::new();
    let mut speed_drift = 0.0f64;

    for i in 0..n {
        let dt = dts[i];
        let (typ, lt) = kinds[i];
        t_acc += dt;
        let pos = |ss: f64| {
            if closed {
                ring_point_at(dense, arcs, ss)
            } else {
                // 1:1 真实米制步进，避免坐标被拉伸导致 GPS 实际移动配速失真
                open_point_at(dense, arcs, ss)
            }
        };
        let d_step = disp_of[i];
        s += direction * d_step;
        let (bx, by) = pos(s);
        let x = bx;
        let y = by;
        jx = 0.72 * jx + rng.gauss(0.0, 0.75);
        jy = 0.72 * jy + rng.gauss(0.0, 0.75);
        let px = bx + jx;
        let py = by + jy;
        let rad = round_to(
            if typ == 3 {
                rng.uniform(1.4, 4.2)
            } else {
                rng.uniform(1.4, 2.4)
            },
            2,
        );
        // 全程纯绿连续运动状态：state=1（绝不注入 state=2 静止标签，防止 App 误判为中途暂停）
        let state = 1;
        dist_acc += d_step;
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt += 0.04 * (82.0 - alt) + rng.gauss(0.0, alt_sigma);
        // 生物力学高步频小步幅联动（加入宏观生理节律波，有快有慢：172~192 spm）
        let v_now = speeds[i];
        let _km_done = (t_acc / dur_f) * dist / 1000.0;
        let _fat = route_planner::fatigue_from_km(_km_done);
        let _gait = route_planner::gait(v_now, _fat);
        // 起步慢、前 45 秒平滑爬升至均值，随后自然波动的步频节律
        let warmup_u = (t_acc / 45.0).clamp(0.0, 1.0);
        let warmup_s = 3.0 * warmup_u * warmup_u - 2.0 * warmup_u * warmup_u * warmup_u;
        let warmup_delta = -20.0 * (1.0 - warmup_s);

        let macro_wave = 5.0 * (std::f64::consts::TAU * t_acc / 170.0 + phi1).sin()
            + 3.5 * (std::f64::consts::TAU * t_acc / 95.0 + phi2).cos();
        let u_artifact = rng.random();
        let u_prog = (t_acc / dur_f).clamp(0.0, 1.0);
        let u_mid: f64 = (u_prog - 0.58) / 0.055;
        let walk_dip = if allow_mid_slowdown && n >= 20 && u_mid.abs() < 2.5 {
            (-0.5f64 * u_mid * u_mid).exp()
        } else {
            0.0
        };
        let run_cad = (183.0 + warmup_delta + macro_wave + rng.gauss(0.0, 3.2)).clamp(152.0, 198.0);
        let walk_cad = (118.0 + rng.gauss(0.0, 3.0)).clamp(108.0, 128.0);
        let cad = if walk_dip > 0.05 {
            run_cad * (1.0 - 0.85 * walk_dip) + walk_cad * (0.85 * walk_dip)
        } else if u_artifact < 0.008 {
            rng.uniform(215.0, 255.0) // 口袋轻微晃动瞬时小尖峰
        } else {
            run_cad
        };
        let stride = (v_now * 60.0 / cad).clamp(0.32, 1.25);
        let cad = v_now * 60.0 / stride;
        steps_acc += cad / 60.0 * dt;
        let nxt = pos(s + direction * 2.0);
        let brg =
            ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 35.0)).rem_euclid(360.0);
        let avg_sp = round_to((d_step / dt.max(0.1)).clamp(SPEED_FLOOR, SPEED_CEIL), 4);
        // 关键规范：allLocJson 中的 speed 为高德/百度定位 SDK 采集的原生 GPS 时速（km/h）！
        // 当时速在正常跑步区间（8.0~15.0 km/h）时，官方 App 地图画线判定为有效跑动并渲染为纯绿轨迹；
        // 杜绝将其误填为分/公里配速（如 6.3~7.5），否则会被地图 SDK 误判为 <7km/h 的走步慢行并涂成灰色暂停！
        let kmh = avg_sp * 3.6;
        let speed_sigma = (kmh * 0.03).max(0.02);
        speed_drift = 0.82 * speed_drift + rng.gauss(0.0, speed_sigma);
        let gps_speed = if i <= 1 {
            0.0
        } else {
            round_to((kmh + speed_drift).clamp(SPEED_FLOOR * 3.6, SPEED_CEIL * 3.6), 4)
        };

        ten_t += dt;
        ten_d += speeds[i] * dt;
        ten_st += cad / 60.0 * dt;
        while ten_t >= 10.0 {
            let k = 10.0 / ten_t; // 只取前 10s 的量，余量留给下一窗
            let raw_d = ten_d * k;
            let raw_st = ten_st * k;
            // 严格对齐学校配速要求（2'00"~10'00"，即 10s 内跑 16.67~83.33m），
            // 并约束在用户要求的 4'00"~10'00" 窗口（10s 内 17.50~41.00m），彻底消除掉入 72'00" 的断崖红线！
            let out_d = raw_d.clamp(17.50, 41.00);
            let out_st = raw_st.clamp(25.0, 45.0);
            ten_speed.push(TenWindow { time: 10, value: round_to(out_d, 2) });
            ten_steps.push(TenWindow { time: 10, value: round_to(out_st, 0) });
            ten_t -= 10.0;
            ten_d -= raw_d;
            ten_st -= raw_st;
        }
        locs.push(GenPoint {
            id: i as i64 + 1,
            flag: start_ms,
            lat: -1.0,
            lng: -1.0,
            gLat: round_to(lat, 7),
            gLng: round_to(lng, 7),
            speed: round_to(gps_speed, 4),
            avgSpeed: avg_sp,
            radius: rad,
            accuracy: rad,
            ptype: typ,
            locType: lt,
            hasAltitude: true,
            totalTime: round_to(t_acc, 0) as i64,
            totalDis: round_to(dist_acc, 4),
            validDis: round_to(dist_acc, 4),
            validTime: round_to(t_acc, 0) as i64,
            steps: steps_acc as i64,
            stepDistance: 0.0,
            gainTime: fmt_gain_time(start_ms + (t_acc * 1000.0) as i64),
            gainTimeMs: start_ms + (t_acc * 1000.0) as i64,
            queueNum: 0,
            coorType: "gcj02".into(),
            bdA: round_to(alt, 2),
            bdD: round_to(brg, 2),
            bdS: round_to(avg_sp, 4),
            bdG: 1, // 全程 GPS 强信号有效，绝不注入 -1 丢星状态码，防止 App 将路段渲染为灰色线
            count: rng.randint(20, 88),
            dtr: 0.0,
            state,
            locationId: String::new(),
        });
    }
    if ten_t > 0.5 {
        let last_time = 1.max(ten_t.round() as i64);
        let out_d = (speeds.last().copied().unwrap_or(base) * ten_t).clamp(1.75 * ten_t, 4.10 * ten_t);
        let out_st = (180.0 / 60.0 * ten_t).round().max(1.0);
        ten_speed.push(TenWindow { time: last_time, value: round_to(out_d, 2) });
        ten_steps.push(TenWindow { time: last_time, value: round_to(out_st, 0) });
    }
    // 消除离散时间窗锯齿与大幅跳变，生成平稳连续、符合自然生理律动的 10 秒配速曲线
    let allow_mid_slowdown = crate::api::model::load_config().allow_mid_slowdown;
    smooth_ten_sec_windows(
        dur,
        dist,
        seed,
        allow_mid_slowdown,
        &mut ten_speed,
        &mut ten_steps,
    );

    let mut segments: Vec<Segment> = Vec::new();
    let (mut seg_t, mut seg_d, mut seg_n) = (0.0f64, 0.0f64, 0i64);
    let mut seg_v: Vec<f64> = Vec::new();
    for i in 0..n {
        seg_t += dts[i];
        seg_d += seg_dist[i];
        seg_v.push(speeds[i]);
        seg_n += 1;
        if seg_t >= 60.0 || i == n - 1 {
            segments.push(Segment {
                totalTime: round_to(seg_t, 0) as i64,
                distance: round_to(seg_d, 0) as i64,
                startTime: round_to((times[i] - seg_t) * 1000.0, 0) as i64,
                endTime: round_to(times[i] * 1000.0, 0) as i64,
                avgSpeed: round_to(seg_v.iter().sum::<f64>() / seg_v.len() as f64, 3),
                avgStep: round_to(steps_acc / 1.0f64.max(t_acc) * 60.0, 0) as i64,
                state: 0,
            });
            seg_t = 0.0;
            seg_d = 0.0;
            seg_v.clear();
            seg_n = 0;
        }
    }
    let _ = seg_n;

    apply_post_fixes(&mut locs, &mut rng, start_ms);

    // 点位吸附策略优化：
    // - 自定义手绘/操场规划模式（pin_start=true）：不强行拽动坐标！路线在打卡点 20m 范围内即可达标，
    //   彻底杜绝强行把点拉向道路旁 15 米造成尖锐折返三角刺被高德/App 判定为异常漂移灰线；
    // - 经典自动拟合模式（pin_start=false）：在 8m 范围内做贴合，满足测试要求。
    let snap_limit = if pin_start { 0.0 } else { 8.0 * 8.0 };
    for pl in points_bd {
        let mut best_i = None;
        let mut best_d = 1e18f64;
        for (i, q) in locs.iter().enumerate() {
            if i <= 1 || i >= locs.len() - 1 {
                continue;
            }
            let dd = ((q.gLat - pl.0) * MET_PER_DEG_LAT).powi(2)
                + ((q.gLng - pl.1) * MET_PER_DEG_LNG).powi(2);
            if dd < best_d {
                best_d = dd;
                best_i = Some(i);
            }
        }
        if let Some(i) = best_i {
            if best_d < snap_limit {
                let d_prev = ((pl.0 - locs[i - 1].gLat) * MET_PER_DEG_LAT)
                    .hypot((pl.1 - locs[i - 1].gLng) * MET_PER_DEG_LNG);
                let d_next = ((locs[i + 1].gLat - pl.0) * MET_PER_DEG_LAT)
                    .hypot((locs[i + 1].gLng - pl.1) * MET_PER_DEG_LNG);
                if d_prev >= 1.5 && d_next >= 1.5 {
                    locs[i].gLat = round_to(pl.0, 7);
                    locs[i].gLng = round_to(pl.1, 7);
                }
            }
        }
    }

    let total_dis = round_to(dist, 3);
    Track {
        totalTime: round_to(t_acc, 0) as i64,
        totalDistance: total_dis,
        validDistance: total_dis,
        validTime: round_to(t_acc, 0) as i64,
        startTime: start_ms,
        startLatitude: locs[0].gLat,
        startLongitude: locs[0].gLng,
        totalSteps: steps_acc as i64,
        locations: locs,
        speedPerTenSec: ten_speed,
        stepsPerTenSec: ten_steps,
        segments,
    }
}
