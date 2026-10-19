//! 轨迹后处理：
//! 结尾断崖（最后正常点压速 1.2-2.6km/h、时间延伸 6-14s）、起终点哨兵
//! （索引1 type=5 全零 / 末点 type=6 / 首点 type∈{0,7}）、后半段 type=1/2 散布、
//! 索引2 avgSpeed 边界修正、首点速度取首个非零值。

use super::geom::{fmt_gain_time, round_to, Rng};
use super::model::GenPoint;

pub fn apply_post_fixes(locs: &mut [GenPoint], rng: &mut Rng, start_ms: i64) {
    // 结尾断崖：最后正常点压速 + 时间自然延伸 6-14s
    if locs.len() >= 4 {
        let mut ci = locs.len() - 1;
        while ci > 0 && locs[ci].ptype == -1 {
            ci -= 1;
        }
        if ci > 1 {
            let stop_t = locs[ci].totalTime + rng.randint(6, 14);
            locs[ci].totalTime = stop_t;
            locs[ci].validTime = stop_t;
            // 收尾减速仍须落在有效配速窗口内（2.0-2.6 m/s ≈ 7.2-9.4 km/h）
            let stop_ms = round_to(rng.uniform(2.0, 2.6), 4);
            locs[ci].speed = round_to(stop_ms * 3.6, 4);
            locs[ci].avgSpeed = stop_ms;
            locs[ci].bdS = round_to(rng.uniform(0.05, 0.15), 3);
            locs[ci].gainTime = fmt_gain_time(start_ms + stop_t * 1000);
            locs[ci].gainTimeMs = start_ms + stop_t * 1000;
            // totalDis 保持（App 过滤条件 totalDis>0）
        }
    }
    if locs.len() > 2 {
        // 起点哨兵（type=5→setStartPoint）：全零累计 + state=1，坐标与首点严格重合，避免起跑点间产生假性零速灰线
        let (p0_lat, p0_lng) = (locs[0].gLat, locs[0].gLng);
        let sp = &mut locs[1];
        sp.gLat = p0_lat;
        sp.gLng = p0_lng;
        sp.ptype = 5;
        sp.state = 1;
        sp.locType = 1;
        sp.radius = round_to(rng.uniform(2.0, 3.5), 2);
        sp.speed = 0.0;
        sp.avgSpeed = 0.0;
        sp.bdS = 0.0;
        sp.totalDis = 0.0;
        sp.validDis = 0.0;
        sp.totalTime = 0;
        sp.validTime = 0;
        sp.steps = 0;
        // 终点哨兵
        let last = locs.len() - 1;
        let e = &mut locs[last];
        e.ptype = 6;
        e.locType = 1;
        e.radius = round_to(rng.uniform(1.5, 3.0), 2);
    }
    // 边界修正①：索引2 首个真实点 avgSpeed 与 speed 平滑承接，消除断崖
    if locs.len() > 3 {
        let next_sp = locs[3].avgSpeed;
        let p2 = &mut locs[2];
        if p2.avgSpeed <= 0.0 || p2.avgSpeed < crate::track::generator::SPEED_FLOOR {
            p2.avgSpeed = next_sp;
            p2.speed = round_to(1000.0 / (next_sp * 60.0), 4);
        }
    }
    // 首点 = 起点：totalTime=0、type=0（标准正常绿点，移除 7 规避部分机型起步灰段）、速度取首个非零真实点
    if !locs.is_empty() {
        locs[0].totalTime = 0;
        locs[0].validTime = 0;
        locs[0].state = 1;
        locs[0].ptype = 0;
        locs[0].locType = 1;
        locs[0].radius = round_to(rng.uniform(1.5, 3.0), 2);
        for j in 1..locs.len() {
            if locs[j].avgSpeed > 0.0 {
                locs[0].avgSpeed = locs[j].avgSpeed;
                locs[0].speed = locs[j].speed;
                locs[0].bdS = locs[j].bdS;
                break;
            }
        }
    }
}
