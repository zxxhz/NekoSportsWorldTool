//! 真实道路路由轨迹生成器（模式 B）。
//!
//! 与经典模式（`generator::build`）的分工：
//! - 几何源为路网闭环路线（而非 Catmull-Rom 环）；
//! - 配速为运动学速度剖面：OU 均值回归 + 弯道预判刹车（`kinematics::pace_profile`）；
//! - 步频/步幅为生物力学 OU 耦合（`biomech::ou_cadence_series` + `gait`）；
//! - GPS 抖动为 AR(1) 相关漂移 + 高斯测量噪声 + 建筑 SDF 衰减（`noise::GpsJitter`）。
//!
//! 协议字段/哨兵/断崖/吸附/10s 窗与经典模式完全一致。

use super::generator::{fit_speeds_bounded, SPEED_CEIL, SPEED_FLOOR};
use super::geom::{
    fmt_gain_time, make_ordered_path, open_point_at, ring_point_at, round_to, to_bd,
    wgs84_to_bd09, Rng, MET_PER_DEG_LAT, MET_PER_DEG_LNG,
};
use super::model::{GenPoint, Segment, TenWindow, Track};
use super::postfix::apply_post_fixes;
pub use route_planner::CustomCloseMode;
use route_planner::{
    fatigue_from_km, gait, ou_cadence_series, pace_profile, plan_custom_route, plan_route_split,
    point_in_polygon, Coord, GpsJitter, KinParams, RoadGraph, Route, RouteOptions, Sdf,
};

/// 路线算法模式（三选一）。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum RouteMode {
    /// 经典打卡点环（Catmull-Rom）。
    #[default]
    Legacy,
    /// 真实道路拓扑路由（自动闭环）。
    Road,
    /// 地图自定义规划路径（按用户放置的途经点路由）。
    Custom,
}

impl RouteMode {
    pub fn from_str(s: &str) -> Self {
        match s {
            "road" => RouteMode::Road,
            "custom" => RouteMode::Custom,
            _ => RouteMode::Legacy,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            RouteMode::Legacy => "legacy",
            RouteMode::Road => "road",
            RouteMode::Custom => "custom",
        }
    }
}

/// 从本地文件加载 OSM 路网。
pub fn load_network_path(path: &str) -> Result<RoadGraph, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("读取 {path} 失败: {e}"))?;
    route_planner::load_osm(&bytes)
}

/// 路网预览计划（供地图显示）：对齐后的路网/建筑/水绿地/操场/标签 + 生成的路线（BD 系）。
#[derive(Clone, Default)]
pub struct RoadPlan {
    pub route: Vec<(f64, f64)>,
    pub length_m: f64,
    pub edges: Vec<Vec<(f64, f64)>>,
    pub buildings: Vec<Vec<(f64, f64)>>,
    pub waters: Vec<Vec<(f64, f64)>>,
    pub greens: Vec<Vec<(f64, f64)>>,
    pub pitches: Vec<Vec<(f64, f64)>>,
    /// `(lat, lon, name, kind)`
    pub labels: Vec<(f64, f64, String, String)>,
    pub checkpoints: Vec<(f64, f64)>,
    pub fences: Vec<Vec<(f64, f64)>>,
    pub route_error: Option<String>,
}

fn extract_map_features(
    g: &RoadGraph,
) -> (
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<(f64, f64, String, String)>,
) {
    let to_rings = |rings: &[Vec<Coord>]| -> Vec<Vec<(f64, f64)>> {
        rings
            .iter()
            .map(|r| r.iter().map(|c| (c.lat, c.lon)).collect())
            .collect()
    };
    let edges = to_rings(&g.edges_deg());
    let buildings = to_rings(&g.buildings);
    let waters = to_rings(&g.waters);
    let greens = to_rings(&g.greens);
    let pitches = to_rings(&g.pitches);
    let labels = g
        .labels
        .iter()
        .map(|l| (l.coord.lat, l.coord.lon, l.name.clone(), l.kind.clone()))
        .collect();
    (edges, buildings, waters, greens, pitches, labels)
}

/// 用围栏（BD 系，与打卡点同系）裁剪路网：仅保留围栏内道路；裁剪后无路则退回原网。
pub fn apply_fences(net: &RoadGraph, fences: &[Vec<(f64, f64)>]) -> RoadGraph {
    if fences.is_empty() {
        return net.clone();
    }
    let mut g = net.clone();
    let polys: Vec<Vec<Coord>> = fences
        .iter()
        .map(|r| r.iter().map(|p| Coord::new(p.1, p.0)).collect())
        .collect();
    g.retain_inside_any(&polys);
    // 进一步裁剪：移除「穿越围栏边界」的边（端点在内但中段跑出围栏，如跨校区道路）。
    g.graph.retain_edges(|graph, e| {
        let (a, b) = graph.edge_endpoints(e).unwrap();
        edge_inside(&polys, graph[a].coord, graph[b].coord)
    });
    g.rebuild_index();
    if g.graph.edge_count() == 0 {
        return net.clone();
    }
    g
}

/// 边是否整体落在任一围栏内：沿边等距采样，所有采样点（含端点）必须在围栏内。
fn edge_inside(polys: &[Vec<Coord>], a: Coord, b: Coord) -> bool {
    const SEGS: usize = 8;
    for k in 0..=SEGS {
        let t = k as f64 / SEGS as f64;
        let p = Coord::new(a.lon + (b.lon - a.lon) * t, a.lat + (b.lat - a.lat) * t);
        if !polys.iter().any(|poly| point_in_polygon(poly, p)) {
            return false;
        }
    }
    true
}

/// 生成路网预览（对齐 + 围栏裁剪 + 闭环路由），不产生协议字段。
///
/// 预览阶段拿不到 policy 必经点，故仅做「软引导 + 吸附」，不强制经过任何打卡点。
pub fn plan_road_view(
    net: &RoadGraph,
    points_bd: &[(f64, f64)],
    dist: f64,
    seed: u64,
    fences: &[Vec<(f64, f64)>],
) -> Result<RoadPlan, String> {
    let mut g = net.clone();
    align_network(&mut g);
    g = apply_fences(&g, fences);
    let ordered = radial_order(points_bd);
    let waypoints: Vec<Coord> = ordered.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let opts = RouteOptions::default();
    let route = plan_route_split(&g, &waypoints, &[], dist, seed, &opts)?;
    let route_pts: Vec<(f64, f64)> = route.points.iter().map(|p| (p.lat, p.lon)).collect();
    let (edges, buildings, waters, greens, pitches, labels) = extract_map_features(&g);
    Ok(RoadPlan {
        route: route_pts,
        length_m: route.length_m,
        edges,
        buildings,
        waters,
        greens,
        pitches,
        labels,
        checkpoints: ordered,
        fences: fences.to_vec(),
        route_error: None,
    })
}

/// 生成自定义路径规划预览：
/// - 若 `snap_road = true` 且已加载 OSM 路网，则对齐到 BD-09（可选围栏裁剪）并沿真实道路按序连接 `waypoints_bd`；
/// - 若 `snap_road = false`（自由手绘 / 操场跑圈模式）或未加载 OSM 路网，则严格按用户点击的 `waypoints_bd` 原始位置拟合平滑闭合环或折返路线（仍保留底图道路/建筑/操场显示，不强制吸附到外部道路）；
/// - 即使途经点不足或寻路报错，仍返回完整的底图要素供交互点击。
#[allow(clippy::too_many_arguments)]
pub fn plan_custom_view(
    net: Option<&RoadGraph>,
    waypoints_bd: &[(f64, f64)],
    checkpoints_bd: &[(f64, f64)],
    fences: &[Vec<(f64, f64)>],
    clip_fence: bool,
    snap_road: bool,
    close_mode: CustomCloseMode,
    dist_m: f64,
    seed: u64,
) -> RoadPlan {
    let ordered_cp = radial_order(checkpoints_bd);
    let (edges, buildings, waters, greens, pitches, labels, aligned_net) =
        if let Some(raw_net) = net {
            let mut g = raw_net.clone();
            align_network(&mut g);
            if clip_fence {
                g = apply_fences(&g, fences);
            }
            let (e, b, w, gr, p, l) = extract_map_features(&g);
            (e, b, w, gr, p, l, Some(g))
        } else {
            (
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
            )
        };

    // 模式 1：开启了「沿道路寻路」且已加载 OSM 路网
    if snap_road {
        if let Some(g) = aligned_net {
            if waypoints_bd.is_empty() {
                return RoadPlan {
                    route: Vec::new(),
                    length_m: 0.0,
                    edges,
                    buildings,
                    waters,
                    greens,
                    pitches,
                    labels,
                    checkpoints: ordered_cp,
                    fences: fences.to_vec(),
                    route_error: Some(
                        "左键点击地图添加起点与途经点（当前为「沿道路寻路」，若进操场可切换为「自由手绘/操场」）"
                            .into(),
                    ),
                };
            }
            let wp_coords: Vec<Coord> = waypoints_bd
                .iter()
                .map(|&(lat, lon)| Coord::new(lon, lat))
                .collect();
            let opts = RouteOptions::default();
            return match plan_custom_route(&g, &wp_coords, close_mode, dist_m, seed, &opts) {
                Ok(route) => {
                    let route_pts: Vec<(f64, f64)> =
                        route.points.iter().map(|p| (p.lat, p.lon)).collect();
                    RoadPlan {
                        route: route_pts,
                        length_m: route.length_m,
                        edges,
                        buildings,
                        waters,
                        greens,
                        pitches,
                        labels,
                        checkpoints: ordered_cp,
                        fences: fences.to_vec(),
                        route_error: None,
                    }
                }
                Err(e) => RoadPlan {
                    route: waypoints_bd.to_vec(),
                    length_m: 0.0,
                    edges,
                    buildings,
                    waters,
                    greens,
                    pitches,
                    labels,
                    checkpoints: ordered_cp,
                    fences: fences.to_vec(),
                    route_error: Some(e),
                },
            };
        }
    }

    // 模式 2：自由手绘 / 操场跑圈模式（不强制吸附道路，严格经过用户点击的每个点）
    if waypoints_bd.len() < 2 {
        return RoadPlan {
            route: waypoints_bd.to_vec(),
            length_m: 0.0,
            edges,
            buildings,
            waters,
            greens,
            pitches,
            labels,
            checkpoints: ordered_cp,
            fences: fences.to_vec(),
            route_error: Some(
                "左键点击地图添加起点与途经点（可在中间点选操场位置后点「+追加操场跑圈」，再去往下一途经点）"
                    .into(),
            ),
        };
    }
    let (dense, arcs, (cx, cy)) = make_ordered_path(waypoints_bd, close_mode);
    let mut route_pts: Vec<(f64, f64)> = dense
        .iter()
        .map(|&(x, y)| to_bd(x, y, cx, cy))
        .collect();
    if close_mode.is_closed() {
        if let Some(&first) = route_pts.first() {
            route_pts.push(first);
        }
    }
    let length_m = *arcs.last().unwrap_or(&0.0);
    RoadPlan {
        route: route_pts,
        length_m,
        edges,
        buildings,
        waters,
        greens,
        pitches,
        labels,
        checkpoints: ordered_cp,
        fences: fences.to_vec(),
        route_error: None,
    }
}

/// 在当前路径序列中追加 `laps` 圈标准 400m 田径操场跑道（保留已有前后开环路段！）：
/// - 若 `waypoints` 非空，则将最后一个点视为用户刚点下的「操场中心定位点」（弹出并替换为跑道圈点列），
///   且自动从离前一个路径点最近的跑道切入点进场，跑完 `laps` 圈后停在该切出点，方便后续继续点选出操场后的第 2 个目标点位；
/// - 若 `waypoints` 为空，则以 `fallback_center` 为中心生成 `laps` 圈跑道。
pub fn append_playground_laps(
    waypoints: &mut Vec<(f64, f64)>,
    fallback_center: (f64, f64),
    laps: usize,
) {
    let laps = laps.clamp(1, 20);
    let center = waypoints.pop().unwrap_or(fallback_center);
    let base_oval = make_playground_oval(&[center], center);
    if base_oval.is_empty() {
        return;
    }

    // 若前面已有路径点（例如起点/第一打卡点），选择离前一点最近的跑道点作为切入/切出点
    let start_idx = if let Some(&prev) = waypoints.last() {
        base_oval
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = ((a.0 - prev.0) * MET_PER_DEG_LAT).powi(2)
                    + ((a.1 - prev.1) * MET_PER_DEG_LNG).powi(2);
                let db = ((b.0 - prev.0) * MET_PER_DEG_LAT).powi(2)
                    + ((b.1 - prev.1) * MET_PER_DEG_LNG).powi(2);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(idx, _)| idx)
            .unwrap_or(0)
    } else {
        0
    };

    let m = base_oval.len();
    for _ in 0..laps {
        for k in 0..m {
            waypoints.push(base_oval[(start_idx + k) % m]);
        }
    }
    // 闭合最后一圈回到切出点，方便无缝继续跑向下一个点位
    waypoints.push(base_oval[start_idx]);
}

/// 将用户手动在操场/环线上点出的最近一圈路径点再原地复制追加一圈（保留前面的进场路段）。
///
/// 自动在末尾 3..=16 个点中寻找与末点距离最近（或默认取最近 4~8 个点）的一圈并追加。
pub fn repeat_last_loop(waypoints: &mut Vec<(f64, f64)>) -> bool {
    let n = waypoints.len();
    if n < 3 {
        return false;
    }
    let last = waypoints[n - 1];
    // 在倒数第 3 到倒数第 14 个点之间寻找与末点距离 < 90m 的闭环起点
    let max_back = (n - 1).min(14);
    let mut loop_start: Option<usize> = None;
    let mut best_d = 90.0 * 90.0;
    for back in 3..=max_back {
        let idx = n - 1 - back;
        let p = waypoints[idx];
        let d2 = ((p.0 - last.0) * MET_PER_DEG_LAT).powi(2)
            + ((p.1 - last.1) * MET_PER_DEG_LNG).powi(2);
        if d2 < best_d {
            best_d = d2;
            loop_start = Some(idx);
        }
    }

    if let Some(start_idx) = loop_start {
        // 已闭合一圈（start_idx .. n-1），复制 (start_idx + 1)..n 追加一圈
        let seg: Vec<(f64, f64)> = waypoints[start_idx + 1..n].to_vec();
        waypoints.extend(seg);
        true
    } else {
        // 用户刚绕操场点了 k 个点尚未闭合回本圈起点：取最近 min(n, 6) 个点先闭合再绕一圈
        let k = n.min(6);
        let start_idx = n - k;
        let mut seg: Vec<(f64, f64)> = waypoints[start_idx..n].to_vec();
        seg.push(waypoints[start_idx]);
        waypoints.extend(seg);
        true
    }
}

/// 跑圈入点模式
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum LoopEntryMode {
    #[default]
    AutoClosest, // 自动选择离上一路径点最近的点入圈（无缝进场）
    South,       // 南侧直道
    North,       // 北侧直道
    East,        // 东侧弯道
    West,        // 西侧弯道
}

impl LoopEntryMode {
    pub fn from_str(s: &str) -> Self {
        match s {
            "south" => LoopEntryMode::South,
            "north" => LoopEntryMode::North,
            "east" => LoopEntryMode::East,
            "west" => LoopEntryMode::West,
            _ => LoopEntryMode::AutoClosest,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            LoopEntryMode::AutoClosest => "auto",
            LoopEntryMode::South => "south",
            LoopEntryMode::North => "north",
            LoopEntryMode::East => "east",
            LoopEntryMode::West => "west",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            LoopEntryMode::AutoClosest => "自动最近入点",
            LoopEntryMode::South => "南侧直道",
            LoopEntryMode::North => "北侧直道",
            LoopEntryMode::East => "东侧弯道",
            LoopEntryMode::West => "西侧弯道",
        }
    }
}

/// 跑圈出点模式
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum LoopExitMode {
    #[default]
    SameAsEntry, // 同入点（跑完整数圈出圈）
    Opposite,    // 对侧出圈（跑 N + 半圈从对侧门出圈）
    North,       // 从北侧出圈
    South,       // 从南侧出圈
    East,        // 从东侧出圈
    West,        // 从西侧出圈
}

impl LoopExitMode {
    pub fn from_str(s: &str) -> Self {
        match s {
            "opposite" => LoopExitMode::Opposite,
            "north" => LoopExitMode::North,
            "south" => LoopExitMode::South,
            "east" => LoopExitMode::East,
            "west" => LoopExitMode::West,
            _ => LoopExitMode::SameAsEntry,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            LoopExitMode::SameAsEntry => "same",
            LoopExitMode::Opposite => "opposite",
            LoopExitMode::North => "north",
            LoopExitMode::South => "south",
            LoopExitMode::East => "east",
            LoopExitMode::West => "west",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            LoopExitMode::SameAsEntry => "同入点（整圈出圈）",
            LoopExitMode::Opposite => "对侧出圈（加半圈）",
            LoopExitMode::North => "从北侧出圈",
            LoopExitMode::South => "从南侧出圈",
            LoopExitMode::East => "从东侧出圈",
            LoopExitMode::West => "从西侧出圈",
        }
    }
}

/// 可调跑圈参数规格
#[derive(Clone, Debug)]
pub struct LoopTrackSpec {
    /// 单圈周长（米）：如 400.0（标准田径场）、250.0、300.0 等
    pub perimeter_m: f64,
    /// 圈数（1..=20）
    pub laps: usize,
    /// 形状：true=标准操场椭圆（两直道+两半圆），false=正圆跑圈
    pub is_oval: bool,
    /// 走向旋转角度（度，0=南北走向长轴，90=东西走向长轴）
    pub rotation_deg: f64,
    /// 转向：true=逆时针左转（标准田径场规则），false=顺时针右转
    pub counter_clockwise: bool,
    /// 入点位置模式
    pub entry_mode: LoopEntryMode,
    /// 出点位置模式
    pub exit_mode: LoopExitMode,
}

impl Default for LoopTrackSpec {
    fn default() -> Self {
        LoopTrackSpec {
            perimeter_m: 400.0,
            laps: 2,
            is_oval: true,
            rotation_deg: 0.0,
            counter_clockwise: true,
            entry_mode: LoopEntryMode::AutoClosest,
            exit_mode: LoopExitMode::SameAsEntry,
        }
    }
}

/// 生成参数化跑圈控制点序列，并返回 `(points, entry_rel_idx, exit_rel_idx)`。
pub fn generate_parametric_loop_points(
    center: (f64, f64),
    spec: &LoopTrackSpec,
    prev_point: Option<(f64, f64)>,
) -> (Vec<(f64, f64)>, usize, usize) {
    let p = spec.perimeter_m.clamp(100.0, 2000.0);
    let laps = spec.laps.clamp(1, 20);

    // 1. 构建单圈 16 个基准局部坐标 (u, v)（u 沿长轴，v 垂直长轴）
    let base_uv: Vec<(f64, f64)> = if spec.is_oval {
        // 标准操场几何比例：2L + 2πR = R * (4.624 + 2π) ≈ 10.907 R
        let r = p / (4.624 + 2.0 * std::f64::consts::PI);
        let h = 1.156 * r; // 半直道长
        vec![
            // 东直道（4 点）
            (-h * 0.5, -r),
            (0.0, -r),
            (h * 0.5, -r),
            (h, -r),
            // 北弯道半圆（4 点）
            (h + r * 0.707, -r * 0.707),
            (h + r, 0.0),
            (h + r * 0.707, r * 0.707),
            (h, r),
            // 西直道（4 点）
            (h * 0.5, r),
            (0.0, r),
            (-h * 0.5, r),
            (-h, r),
            // 南弯道半圆（4 点）
            (-h - r * 0.707, r * 0.707),
            (-h - r, 0.0),
            (-h - r * 0.707, -r * 0.707),
            (-h, -r),
        ]
    } else {
        // 正圆跑道
        let r = p / std::f64::consts::TAU;
        (0..16)
            .map(|k| {
                let ang = k as f64 / 16.0 * std::f64::consts::TAU;
                (r * ang.cos(), r * ang.sin())
            })
            .collect()
    };

    // 2. 旋转并转换为 BD-09 经纬度
    let axis_angle = (spec.rotation_deg + 90.0).to_radians(); // 0° 时长轴指向南北向
    let (sin_a, cos_a) = axis_angle.sin_cos();
    let mut ring_bd: Vec<(f64, f64)> = base_uv
        .iter()
        .map(|&(u, v)| {
            let x = u * cos_a - v * sin_a;
            let y = u * sin_a + v * cos_a;
            to_bd(x, y, center.0, center.1)
        })
        .collect();

    // 逆时针（默认）/ 顺时针
    if !spec.counter_clockwise {
        ring_bd.reverse();
    }

    // 3. 定位入点索引 entry_idx (0..16)
    let entry_idx = match spec.entry_mode {
        LoopEntryMode::AutoClosest => {
            if let Some(prev) = prev_point {
                ring_bd
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        let da = ((a.0 - prev.0) * MET_PER_DEG_LAT).powi(2)
                            + ((a.1 - prev.1) * MET_PER_DEG_LNG).powi(2);
                        let db = ((b.0 - prev.0) * MET_PER_DEG_LAT).powi(2)
                            + ((b.1 - prev.1) * MET_PER_DEG_LNG).powi(2);
                        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|(idx, _)| idx)
                    .unwrap_or(0)
            } else {
                0
            }
        }
        LoopEntryMode::South => {
            // 南侧：min(lat)
            ring_bd
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.0.partial_cmp(&b.0).unwrap())
                .map(|(idx, _)| idx)
                .unwrap_or(0)
        }
        LoopEntryMode::North => {
            // 北侧：max(lat)
            ring_bd
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.0.partial_cmp(&b.0).unwrap())
                .map(|(idx, _)| idx)
                .unwrap_or(0)
        }
        LoopEntryMode::East => {
            // 东侧：max(lon)
            ring_bd
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.1.partial_cmp(&b.1).unwrap())
                .map(|(idx, _)| idx)
                .unwrap_or(0)
        }
        LoopEntryMode::West => {
            // 西侧：min(lon)
            ring_bd
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.1.partial_cmp(&b.1).unwrap())
                .map(|(idx, _)| idx)
                .unwrap_or(0)
        }
    };

    // 4. 定位出点索引 exit_idx (0..16)
    let exit_idx = match spec.exit_mode {
        LoopExitMode::SameAsEntry => entry_idx,
        LoopExitMode::Opposite => (entry_idx + 8) % 16,
        LoopExitMode::North => ring_bd
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.0.partial_cmp(&b.0).unwrap())
            .map(|(idx, _)| idx)
            .unwrap_or(entry_idx),
        LoopExitMode::South => ring_bd
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.0.partial_cmp(&b.0).unwrap())
            .map(|(idx, _)| idx)
            .unwrap_or(entry_idx),
        LoopExitMode::East => ring_bd
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.1.partial_cmp(&b.1).unwrap())
            .map(|(idx, _)| idx)
            .unwrap_or(entry_idx),
        LoopExitMode::West => ring_bd
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.1.partial_cmp(&b.1).unwrap())
            .map(|(idx, _)| idx)
            .unwrap_or(entry_idx),
    };

    // 5. 组装有序路径序列：从 entry_idx 开始，跑 laps 整圈，再跑 extra 弧段直到 exit_idx
    let m = ring_bd.len();
    let mut out_pts = Vec::new();
    for _ in 0..laps {
        for k in 0..m {
            out_pts.push(ring_bd[(entry_idx + k) % m]);
        }
    }
    let extra_steps = (exit_idx + m - entry_idx) % m;
    for k in 0..=extra_steps {
        out_pts.push(ring_bd[(entry_idx + k) % m]);
    }

    let rel_entry_idx = 0;
    let rel_exit_idx = out_pts.len() - 1;
    (out_pts, rel_entry_idx, rel_exit_idx)
}

/// 在已有路径末尾无缝插入一个可调跑圈（保留前面路径！）：
/// - 将最后一个点视为跑圈中心（弹出并替换为跑圈点列），从离上一路点最近的「入点」进场，跑足指定圈数后停在指定的「出点」；
/// - 之后继续点击地图即可直接从「出点」接上后续的打卡点或终点。
pub fn append_custom_loop_track(
    waypoints: &mut Vec<(f64, f64)>,
    fallback_center: (f64, f64),
    spec: &LoopTrackSpec,
) -> (usize, usize) {
    let center = waypoints.pop().unwrap_or(fallback_center);
    let prev = waypoints.last().copied();
    let base_start = waypoints.len();
    let (loop_pts, rel_entry, rel_exit) = generate_parametric_loop_points(center, spec, prev);
    waypoints.extend(loop_pts);
    (base_start + rel_entry, base_start + rel_exit)
}
/// - 若 `existing_wps` 有 1 个点：以该点为操场中心，默认南北向；
/// - 若 `existing_wps` 有 2 个点：以两点中点为操场中心，两点连线方向为操场长轴方向；
/// - 其他情况：以 `fallback_center` 为操场中心，默认南北向。
pub fn make_playground_oval(
    existing_wps: &[(f64, f64)],
    fallback_center: (f64, f64),
) -> Vec<(f64, f64)> {
    let (center, axis_angle) = match existing_wps.len() {
        1 => (existing_wps[0], std::f64::consts::FRAC_PI_2),
        2 => {
            let a = existing_wps[0];
            let b = existing_wps[1];
            let mid = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
            let dx = (b.1 - a.1) * MET_PER_DEG_LNG;
            let dy = (b.0 - a.0) * MET_PER_DEG_LAT;
            let ang = if dx.hypot(dy) > 5.0 {
                dy.atan2(dx)
            } else {
                std::f64::consts::FRAC_PI_2
            };
            (mid, ang)
        }
        _ => (fallback_center, std::f64::consts::FRAC_PI_2),
    };

    // 标准 400m 跑道：两条直道各 84.39m（半长 42.2m），两端半圆半径 36.5m
    let half_straight = 42.2f64;
    let radius = 36.5f64;
    let local_pts: [(f64, f64); 12] = [
        // 东侧/右侧直道（南→北）
        (-half_straight, -radius),
        (0.0, -radius),
        (half_straight, -radius),
        // 北侧弯道（3 个采样点）
        (
            half_straight + radius * 0.707,
            -radius * 0.707,
        ),
        (half_straight + radius, 0.0),
        (
            half_straight + radius * 0.707,
            radius * 0.707,
        ),
        // 西侧/左侧直道（北→南）
        (half_straight, radius),
        (0.0, radius),
        (-half_straight, radius),
        // 南侧弯道（3 个采样点）
        (
            -half_straight - radius * 0.707,
            radius * 0.707,
        ),
        (-half_straight - radius, 0.0),
        (
            -half_straight - radius * 0.707,
            -radius * 0.707,
        ),
    ];

    let (sin_a, cos_a) = axis_angle.sin_cos();
    local_pts
        .iter()
        .map(|&(u, v)| {
            // u 沿长轴方向，v 垂直于长轴
            let x = u * cos_a - v * sin_a;
            let y = u * sin_a + v * cos_a;
            to_bd(x, y, center.0, center.1)
        })
        .collect()
}

/// 将坐标点吸附到对齐后的 BD-09 路网最近道路上（距离阈值内生效）。
pub fn snap_point_to_aligned_road(
    net: &RoadGraph,
    fences: &[Vec<(f64, f64)>],
    clip_fence: bool,
    lat: f64,
    lon: f64,
) -> Option<((f64, f64), f64)> {
    let mut g = net.clone();
    align_network(&mut g);
    if clip_fence {
        g = apply_fences(&g, fences);
    }
    let (snapped, dist_m) = g.snap_to_road(Coord::new(lon, lat))?;
    Some(((snapped.lat, snapped.lon), dist_m))
}

/// 将 WGS84 路网逐节点转换为 BD-09（与打卡点同系）。
///
/// OSM 为 WGS84，打卡点为 BD-09。两者不是常数平移关系（境内 GCJ/BD 偏移随位置
/// 变化），故用标准 WGS84→GCJ-02→BD-09 变换逐节点转换，校园尺度下与真实道路
/// 对齐（优于旧版质心常数平移的米级~数十米残差）。
pub fn align_network(net: &mut RoadGraph) {
    if net.graph.node_count() == 0 {
        return;
    }
    for node in net.graph.node_weights_mut() {
        let (lat, lon) = wgs84_to_bd09(node.coord.lat, node.coord.lon);
        node.coord.lat = lat;
        node.coord.lon = lon;
    }
    for ring in net.buildings.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for ring in net.waters.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for ring in net.greens.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for ring in net.pitches.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for lb in net.labels.iter_mut() {
        let (lat, lon) = wgs84_to_bd09(lb.coord.lat, lb.coord.lon);
        lb.coord.lat = lat;
        lb.coord.lon = lon;
    }
    net.compute_anchor();
    net.rebuild_index();
}

/// 按质心角度排序打卡点，形成自然环序。
pub fn radial_order(points_bd: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = points_bd.len();
    if n == 0 {
        return Vec::new();
    }
    let (cx, cy) = (
        points_bd.iter().map(|q| q.0).sum::<f64>() / n as f64,
        points_bd.iter().map(|q| q.1).sum::<f64>() / n as f64,
    );
    let mut ordered = points_bd.to_vec();
    ordered.sort_by(|a, b| {
        (a.0 - cx)
            .atan2(a.1 - cy)
            .partial_cmp(&(b.0 - cx).atan2(b.1 - cy))
            .unwrap()
    });
    ordered
}

/// 路线 → 平面折线 + 弧长表。
///
/// `arcs[k]` 必须与 `dense[k]` 一一对应（`ring_point_at` 按同下标取弧长），
/// 因此直接取每个采样点的累计弧长 `p.s`（`dense` 末点已闭合回起点）。
fn route_ring(route: &Route, c_lat: f64, c_lng: f64) -> (Vec<(f64, f64)>, Vec<f64>) {
    let dense: Vec<(f64, f64)> = route
        .points
        .iter()
        .map(|p| {
            (
                (p.lon - c_lng) * MET_PER_DEG_LNG,
                (p.lat - c_lat) * MET_PER_DEG_LAT,
            )
        })
        .collect();
    let arcs: Vec<f64> = route.points.iter().map(|p| p.s).collect();
    (dense, arcs)
}

/// 模式 B：真实道路拓扑路由轨迹生成。
///
/// `route_bd[0]` 为起点，`route_bd[1..]` 为软引导点（用于方向锚点，不必全经过）；
/// `must_bd` 为强制必经点（按序，可为空，最终必达）；吸附在 `route_bd` 全量上做。
pub fn build_road(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route_bd: &[(f64, f64)],
    must_bd: &[(f64, f64)],
    net: &RoadGraph,
) -> Result<Track, String> {
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;
    let radius = (avg_v * avg_v / a_c_max).max(6.0);
    let opts = RouteOptions {
        a_c_max,
        min_radius_m: radius,
        ..Default::default()
    };

    let waypoints: Vec<Coord> = route_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let must: Vec<Coord> = must_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let route = plan_route_split(net, &waypoints, &must, dist, seed, &opts)?;
    build_track_from_route(
        dist, dur, seed, start_ms, &route, route_bd, route_bd, true, 40.0, net,
    )
}

/// 模式 C（有 OSM 路网）：按用户自定义途经点在真实路网上规划路线并生成完整协议轨迹。
#[allow(clippy::too_many_arguments)]
pub fn build_custom_road(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    waypoints_bd: &[(f64, f64)],
    close_mode: CustomCloseMode,
    snap_bd: &[(f64, f64)],
    net: &RoadGraph,
) -> Result<Track, String> {
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;
    let radius = (avg_v * avg_v / a_c_max).max(6.0);
    let opts = RouteOptions {
        a_c_max,
        min_radius_m: radius,
        ..Default::default()
    };
    let waypoints: Vec<Coord> = waypoints_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let route = plan_custom_route(net, &waypoints, close_mode, dist, seed, &opts)?;
    build_track_from_route(
        dist,
        dur,
        seed,
        start_ms,
        &route,
        waypoints_bd,
        snap_bd,
        close_mode.is_closed(),
        15.0,
        net,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_track_from_route(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route: &Route,
    ref_bd: &[(f64, f64)],
    snap_bd: &[(f64, f64)],
    closed: bool,
    _snap_radius_m: f64,
    net: &RoadGraph,
) -> Result<Track, String> {
    let mut rng = Rng::new(seed);
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;

    let n_pts = ref_bd.len().max(1);
    let (c_lat, c_lng) = (
        ref_bd.iter().map(|q| q.0).sum::<f64>() / n_pts as f64,
        ref_bd.iter().map(|q| q.1).sum::<f64>() / n_pts as f64,
    );
    let (dense, arcs) = route_ring(route, c_lat, c_lng);

    // 时间网格（同经典模式）
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

    // 运动学速度剖面：OU 配速 + 弯道预判刹车 + 围绕目标平均配速温和波动（严守 4~10 分配速窗口）
    let kin = KinParams {
        a_c_max,
        ..Default::default()
    };
    let base_v = avg_v.clamp(SPEED_FLOOR + 0.05, SPEED_CEIL - 0.05);
    let _base_pace_s = 1000.0 / base_v.max(0.1);
    let allow_mid_slowdown = crate::api::model::load_config().allow_mid_slowdown;
    let local_floor = if allow_mid_slowdown {
        SPEED_FLOOR
    } else {
        (base_v * 0.80).max(SPEED_FLOOR)
    };
    let local_ceil = (base_v * 1.20).min(SPEED_CEIL).max(local_floor + 0.05);
    let speed_seed = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut w = pace_profile(
        route,
        base_v,
        &dts,
        dist,
        speed_seed,
        &kin,
        local_floor,
        local_ceil,
    );

    // 点位类型（同经典模式）
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
    let cur_sum: f64 = w.iter().zip(&dts).map(|(x, dt)| x * dt).sum();
    if (cur_sum - dist).abs() > 2.0 {
        fit_speeds_bounded(&mut w, &dts, dist, SPEED_FLOOR, SPEED_CEIL);
    }
    let disp_of: Vec<f64> = (0..n).map(|i| w[i] * dts[i]).collect();
    let seg_dist: Vec<f64> = disp_of.clone();
    let speeds: Vec<f64> = (0..n)
        .map(|i| if w[i] > 0.0 { w[i] } else { base_v })
        .collect();

    // 建筑 SDF（自适应放大漂移）
    let sdf = Sdf::from_buildings(
        &net.buildings,
        |c| {
            [
                (c.lon - c_lng) * MET_PER_DEG_LNG,
                (c.lat - c_lat) * MET_PER_DEG_LAT,
            ]
        },
        2.0,
        30.0,
    );

    // 生物力学：高步频（180~190 spm）小步幅（65~85 cm）耦合，严格满足 v=(cad/60)·stride
    let targets: Vec<f64> = (0..n)
        .map(|i| {
            let km_done = (times[i] / dur_f) * dist / 1000.0;
            gait(speeds[i], fatigue_from_km(km_done)).cadence
        })
        .collect();
    let cadence_seed = seed.wrapping_add(0xC6A4_A793_5BD1_E995);
    let _cadences = ou_cadence_series(&targets, &dts, cadence_seed, 0.25, 2.0, 160.0, 205.0);

    // GPS 抖动：AR(1) 相关漂移 + 高斯测量噪声（SDF 逐点放大）
    let mut jitter = GpsJitter::new(0.72, 0.75, 0.3);

    let mut locs: Vec<GenPoint> = Vec::with_capacity(n);
    let mut s = 0.0f64;
    let mut t_acc = 0.0f64;
    let mut dist_acc = 0.0f64;
    let mut steps_acc = 0.0f64;
    let mut alt = 82.0 + rng.uniform(-1.0, 1.0);
    let n_est = 1.max((dur_f / 5.0) as i64);
    let alt_sigma = rng.uniform(3.8, 6.2) / (0.40 * n_est as f64);
    let mut ten_t = 0.0f64;
    let mut ten_d = 0.0f64;
    let mut ten_st = 0.0f64;
    let mut ten_speed: Vec<TenWindow> = Vec::new();
    let mut ten_steps: Vec<TenWindow> = Vec::new();
    let mut speed_drift = 0.0f64;

    let phi1 = ((seed & 0xFFFF) as f64 / 65535.0) * std::f64::consts::TAU;
    let phi2 = (((seed >> 16) & 0xFFFF) as f64 / 65535.0) * std::f64::consts::TAU;
    let allow_mid_slowdown = crate::api::model::load_config().allow_mid_slowdown;

    for i in 0..n {
        let dt = dts[i];
        let (typ, lt) = kinds[i];
        t_acc += dt;
        let pos = |ss: f64| {
            if closed {
                ring_point_at(&dense, &arcs, ss)
            } else {
                // 1:1 真实米制步进，保持 GPS 空间位移与累计里程配速严格一致
                open_point_at(&dense, &arcs, ss)
            }
        };
        let d_step = disp_of[i];
        s += d_step;
        let (bx, by) = pos(s);
        let x = bx;
        let y = by;
        // AR(1) 相关漂移 + 高斯测量噪声，SDF 自适应放大
        let sc = sdf.sigma_scale([bx, by]);
        let z = [
            rng.gauss(0.0, 1.0),
            rng.gauss(0.0, 1.0),
            rng.gauss(0.0, 1.0),
            rng.gauss(0.0, 1.0),
        ];
        let (jxd, jyd) = jitter.step(sc, z);
        let px = bx + jxd;
        let py = by + jyd;
        let rad = round_to(
            if typ == 3 {
                rng.uniform(1.4, 4.2)
            } else {
                rng.uniform(1.4, 2.4)
            },
            2,
        );
        // 对齐官方真机样本：起步热身 state=1，巡航稳态以 state=2 为主（约 55%），活跃/加速 state=3（约 35%）
        let state = if t_acc < 30.0 {
            1
        } else if rng.random() < 0.35 {
            3
        } else {
            2
        };
        dist_acc += d_step;
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt += 0.04 * (82.0 - alt) + rng.gauss(0.0, alt_sigma);
        // 生物力学联动：增加宏观生理节律波，有快有慢：172~192 spm
        let v_now = speeds[i];
        let _km_done = (t_acc / dur_f) * dist / 1000.0;
        let warmup_u = (t_acc / 45.0).clamp(0.0, 1.0);
        let warmup_s = 3.0 * warmup_u * warmup_u - 2.0 * warmup_u * warmup_u * warmup_u;
        let warmup_delta = -20.0 * (1.0 - warmup_s);

        let macro_wave = 5.0 * (std::f64::consts::TAU * t_acc / 170.0 + phi1).sin()
            + 3.5 * (std::f64::consts::TAU * t_acc / 95.0 + phi2).cos();
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
        } else {
            run_cad
        };
        let stride = (v_now * 60.0 / cad).clamp(0.32, 1.25);
        let cad = v_now * 60.0 / stride;
        steps_acc += cad / 60.0 * dt;
        let nxt = pos(s + 2.0);
        let brg =
            ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 35.0)).rem_euclid(360.0);
        let avg_sp = round_to((d_step / dt.max(0.1)).clamp(SPEED_FLOOR, SPEED_CEIL), 2);
        // 对齐真机样本：allLocJson 中的 speed 为当前瞬时配速（分/公里，如 4.83），avgSpeed 与 bdS 为瞬时米每秒（m/s，如 3.45）
        let pace_min = round_to(1000.0 / (avg_sp * 60.0), 2);
        speed_drift = 0.82 * speed_drift + rng.gauss(0.0, 0.005);
        let point_pace = if i <= 1 {
            0.0
        } else {
            round_to(pace_min + speed_drift, 2)
        };

        ten_t += dt;
        ten_d += speeds[i] * dt;
        ten_st += cad / 60.0 * dt;
        while ten_t >= 10.0 {
            let k = 10.0 / ten_t;
            let raw_d = ten_d * k;
            let raw_st = ten_st * k;
            let out_d = raw_d.clamp(17.50, 41.00);
            let out_st = raw_st.clamp(25.0, 45.0);
            ten_speed.push(TenWindow {
                time: 10,
                value: round_to(out_d, 2),
            });
            ten_steps.push(TenWindow {
                time: 10,
                value: round_to(out_st, 0),
            });
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
            speed: point_pace,
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
            bdS: avg_sp,
            bdG: 1, // 全程 GPS 强信号有效，绝不注入 -1 丢星状态码，防止 App 将路段渲染为灰色线
            count: rng.randint(20, 88),
            dtr: 0.0,
            state,
            locationId: String::new(),
        });
    }
    if ten_t > 0.5 {
        let last_time = 1.max(ten_t.round() as i64);
        let out_d = (speeds.last().copied().unwrap_or(base_v) * ten_t).clamp(1.75 * ten_t, 4.10 * ten_t);
        let out_st = (180.0 / 60.0 * ten_t).round().max(1.0);
        ten_speed.push(TenWindow {
            time: last_time,
            value: round_to(out_d, 2),
        });
        ten_steps.push(TenWindow {
            time: last_time,
            value: round_to(out_st, 0),
        });
    }
    // 消除离散时间窗锯齿与大幅跳变，生成平稳连续、符合自然生理律动的 10 秒配速曲线
    let allow_mid_slowdown = crate::api::model::load_config().allow_mid_slowdown;
    super::generator::smooth_ten_sec_windows(
        dur,
        dist,
        seed,
        allow_mid_slowdown,
        &mut ten_speed,
        &mut ten_steps,
    );

    let mut segments: Vec<Segment> = Vec::new();
    let (mut seg_t, mut seg_d) = (0.0f64, 0.0f64);
    let mut seg_v: Vec<f64> = Vec::new();
    for i in 0..n {
        seg_t += dts[i];
        seg_d += seg_dist[i];
        seg_v.push(speeds[i]);
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
        }
    }

    apply_post_fixes(&mut locs, &mut rng, start_ms);

    // 点位吸附策略优化：开环单程或远离路线时不强行拽动坐标（避免制造横向 15m 死折返被判定为异常漂移灰线），仅在极近距离（<=3.5m）做微幅贴合
    let snap_limit = if !closed { 0.0 } else { 3.5 * 3.5 };
    for pl in snap_bd {
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
    Ok(Track {
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
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn custom_view_and_ordered_track_work_with_and_without_osm() {
        let wps = vec![
            (38.889000, 121.531000),
            (38.890500, 121.531000),
            (38.890500, 121.533000),
        ];
        let view = plan_custom_view(
            None,
            &wps,
            &wps,
            &[],
            false,
            false,
            CustomCloseMode::Loop,
            2000.0,
            7,
        );
        assert!(view.route_error.is_none());
        assert!(view.length_m > 200.0);
        assert!(view.route.len() > 10);

        // 测试生成 400m 标准操场跑道椭圆
        let oval = make_playground_oval(&[(38.889000, 121.531000)], (38.889000, 121.531000));
        assert_eq!(oval.len(), 12);
        let oval_view = plan_custom_view(
            None,
            &oval,
            &[],
            &[],
            false,
            false,
            CustomCloseMode::Loop,
            2000.0,
            7,
        );
        assert!(
            (oval_view.length_m - 400.0).abs() < 45.0,
            "操场跑道单圈长度应接近 400m，实际: {}",
            oval_view.length_m
        );

        let track = crate::track::generator::build_ordered(
            1500.0,
            540,
            7,
            1_700_000_000_000,
            &wps,
            true,
            &wps,
        );
        assert!((track.totalDistance - 1500.0).abs() < 1.0);
        assert!(!track.locations.is_empty());

        // 测试「点位 1 → 插入操场 2 圈 → 点位 2（开环单程模式，首尾不闭合）」
        let start_pt = (38.887000, 121.529000);
        let playground_center = (38.889000, 121.531000);
        let end_pt = (38.891500, 121.534000);
        let mut mixed_wps = vec![start_pt, playground_center];
        append_playground_laps(&mut mixed_wps, playground_center, 2);
        mixed_wps.push(end_pt);
        assert_eq!(mixed_wps.first().copied(), Some(start_pt));
        assert_eq!(mixed_wps.last().copied(), Some(end_pt));

        let open_view = plan_custom_view(
            None,
            &mixed_wps,
            &[start_pt, end_pt],
            &[],
            false,
            false,
            CustomCloseMode::Open,
            1500.0,
            7,
        );
        assert!(open_view.route_error.is_none());
        let first_r = open_view.route.first().unwrap();
        let last_r = open_view.route.last().unwrap();
        assert!(
            (first_r.0 - start_pt.0).abs() < 1e-5 && (first_r.1 - start_pt.1).abs() < 1e-5,
            "开环路线起点应等于点位 1"
        );
        assert!(
            (last_r.0 - end_pt.0).abs() < 1e-5 && (last_r.1 - end_pt.1).abs() < 1e-5,
            "开环路线终点应等于点位 2，不应闭合回点位 1"
        );

        let open_track = crate::track::generator::build_ordered_mode(
            open_view.length_m,
            540,
            7,
            1_700_000_000_000,
            &mixed_wps,
            CustomCloseMode::Open,
            &[start_pt, end_pt],
        );
        let last_loc = open_track.locations.last().unwrap();
        let dist_to_end = (((last_loc.gLat - end_pt.0) * MET_PER_DEG_LAT).powi(2)
            + ((last_loc.gLng - end_pt.1) * MET_PER_DEG_LNG).powi(2))
        .sqrt();
        assert!(
            dist_to_end < 35.0,
            "开环轨迹末点应停在终点附近而非跳回起点，dist_to_end={dist_to_end}m"
        );

        // 测试可调跑圈生成器：自定义周长 300m、跑 2 圈、南侧入圈、北侧出圈（跑 2.5 圈）
        let spec = LoopTrackSpec {
            perimeter_m: 300.0,
            laps: 2,
            is_oval: true,
            rotation_deg: 0.0,
            counter_clockwise: true,
            entry_mode: LoopEntryMode::South,
            exit_mode: LoopExitMode::North,
        };
        let (loop_pts, rel_entry, rel_exit) =
            generate_parametric_loop_points(playground_center, &spec, Some(start_pt));
        assert_eq!(rel_entry, 0);
        assert_eq!(rel_exit, loop_pts.len() - 1);
        // 2 圈整 + 半圈出点弧段 = 2.5 圈，总长应接近 750m
        let (_, arcs_loop, _) = make_ordered_path(&loop_pts, CustomCloseMode::Open);
        let total_loop_len = *arcs_loop.last().unwrap();
        assert!(
            (total_loop_len - 750.0).abs() < 50.0,
            "2.5 圈 300m 跑道总长应接近 750m，实际: {total_loop_len}m"
        );

        // 验证 append_custom_loop_track 无缝拼接到路径序列中
        let mut full_route = vec![start_pt, playground_center];
        let (in_idx, out_idx) =
            append_custom_loop_track(&mut full_route, playground_center, &spec);
        full_route.push(end_pt);
        assert_eq!(full_route[0], start_pt);
        assert_eq!(in_idx, 1);
        assert_eq!(out_idx, full_route.len() - 2);
        assert_eq!(*full_route.last().unwrap(), end_pt);
    }
}
