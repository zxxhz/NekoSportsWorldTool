//! 跑步配速折线图矢量自绘组件。
//!
//! 在开跑提交前预演生成本次跑步的配速微波曲线与每公里分段数据，
//! 严格采用与实际提交相同的 10 秒平滑窗口算法，所见即所得。

use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Vec2};

use super::theme;
use crate::track::generator::{smooth_ten_sec_windows, SPEED_CEIL, SPEED_FLOOR};
use crate::track::model::TenWindow;

/// 单个 10 秒时间窗预演数据
#[derive(Clone, Copy, Debug)]
pub struct PaceWindowPreview {
    pub time_s: f64,
    pub cum_dist_m: f64,
    pub window_dist_m: f64,
    pub pace_s_per_km: f64,
    pub cadence: f64,
    pub stride_cm: f64,
}

/// 单公里分段预演数据
#[derive(Clone, Debug)]
pub struct LapPreview {
    pub lap_num: usize,
    pub dist_m: f64,
    pub dur_s: i64,
    pub pace_s_per_km: f64,
    pub cadence: f64,
    pub stride_cm: f64,
    pub is_full: bool,
}

/// 本次跑步配速全量预演
#[derive(Clone, Debug, Default)]
pub struct PacePreview {
    pub windows: Vec<PaceWindowPreview>,
    pub laps: Vec<LapPreview>,
    pub min_pace_s: f64,
    pub max_pace_s: f64,
    pub avg_pace_s: f64,
    pub total_dis_m: f64,
    pub total_dur_s: i64,
}

/// 格式化配速（如 437s → 7'17"/km）
pub fn fmt_pace_val(pace_s: f64) -> String {
    if pace_s <= 0.0 || !pace_s.is_finite() {
        return "--'--\"".into();
    }
    let m = (pace_s / 60.0).floor() as i64;
    let s = (pace_s % 60.0).round() as i64;
    format!("{m}'{s:02}\"")
}

/// 格式化秒数为 mm:ss
pub fn fmt_time_val(sec: i64) -> String {
    let m = sec / 60;
    let s = sec % 60;
    format!("{m:02}:{s:02}")
}

/// 根据目标总里程、用时和随机种子预计算配速曲线（与实际提交完全一致）
pub fn build_pace_preview(dist_m: f64, dur_s: i64, seed: u64, allow_mid_slowdown: bool) -> PacePreview {
    if dist_m <= 10.0 || dur_s <= 5 {
        return PacePreview::default();
    }

    let m = ((dur_s + 9) / 10).max(1) as usize;
    let mut ten_speed: Vec<TenWindow> = (0..m)
        .map(|_| TenWindow {
            time: 10,
            value: 0.0,
        })
        .collect();
    let mut ten_steps: Vec<TenWindow> = (0..m)
        .map(|_| TenWindow {
            time: 10,
            value: 0.0,
        })
        .collect();

    if dur_s % 10 != 0 {
        let last_t = dur_s % 10;
        ten_speed[m - 1].time = last_t;
        ten_steps[m - 1].time = last_t;
    }

    // 调用与实际提交一致的平滑算法（含中途喘息缓行与末尾防冲刺斜率限制）
    smooth_ten_sec_windows(dur_s, dist_m, seed, allow_mid_slowdown, &mut ten_speed, &mut ten_steps);

    let mut windows = Vec::with_capacity(m);
    let mut cum_d = 0.0f64;
    let mut cum_t = 0i64;
    let mut min_p = f64::INFINITY;
    let mut max_p = 0.0f64;

    for i in 0..m {
        let wt = ten_speed[i].time as f64;
        let d = ten_speed[i].value;
        let st = ten_steps[i].value;
        cum_d += d;
        cum_t += ten_speed[i].time;

        // 换算当前窗口等效配速（秒/公里）
        let v = (d / wt.max(0.1)).clamp(SPEED_FLOOR, SPEED_CEIL);
        let pace_s = 1000.0 / v;
        if pace_s < min_p {
            min_p = pace_s;
        }
        if pace_s > max_p {
            max_p = pace_s;
        }

        let cad = (st / wt.max(0.1)) * 60.0;
        let stride = (d / st.max(1.0)) * 100.0;

        windows.push(PaceWindowPreview {
            time_s: (cum_t as f64 - wt * 0.5).min(dur_s as f64),
            cum_dist_m: cum_d,
            window_dist_m: d,
            pace_s_per_km: pace_s,
            cadence: cad,
            stride_cm: stride,
        });
    }

    let avg_pace = (dur_s as f64 / (dist_m / 1000.0).max(0.001)).clamp(240.0, 600.0);

    // 计算每公里分段数据（每 1000m 一圈）
    let mut laps = Vec::new();
    let mut lap_prev_d = 0.0f64;
    let mut lap_prev_t = 0.0f64;
    let mut lap_prev_st = 0.0f64;
    let mut acc_st = 0.0f64;

    for i in 0..m {
        let w = &windows[i];
        let st = ten_steps[i].value;
        acc_st += st;
        let is_last = i == m - 1;
        let cur_lap_d = w.cum_dist_m - lap_prev_d;

        if cur_lap_d >= 1000.0 || is_last {
            let lap_t = (w.time_s - lap_prev_t).max(1.0);
            let lap_st = (acc_st - lap_prev_st).max(1.0);
            let lap_pace = (lap_t / (cur_lap_d / 1000.0).max(0.001)).clamp(240.0, 600.0);
            let lap_cad = (lap_st / (lap_t / 60.0)).clamp(160.0, 210.0);
            let lap_stride = (cur_lap_d / lap_st) * 100.0;

            laps.push(LapPreview {
                lap_num: laps.len() + 1,
                dist_m: cur_lap_d,
                dur_s: lap_t.round() as i64,
                pace_s_per_km: lap_pace,
                cadence: lap_cad,
                stride_cm: lap_stride,
                is_full: cur_lap_d >= 995.0,
            });

            lap_prev_d = w.cum_dist_m;
            lap_prev_t = w.time_s;
            lap_prev_st = acc_st;
        }
    }

    PacePreview {
        windows,
        laps,
        min_pace_s: min_p,
        max_pace_s: max_p,
        avg_pace_s: avg_pace,
        total_dis_m: dist_m,
        total_dur_s: dur_s,
    }
}

/// 绘制配速折线图卡片（含交互光标、S 曲线微波填充、公里分段标签）
pub fn draw_pace_preview_card(ui: &mut egui::Ui, preview: &PacePreview) {
    if preview.windows.is_empty() {
        return;
    }

    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.colored_label(
                theme::ok(),
                "📈 配速折线图预演（10秒时间窗 · 真实 S 曲线微波）",
            );
            ui.add_space(8.0);
            ui.colored_label(
                theme::plain(),
                format!("均速: {}/km", fmt_pace_val(preview.avg_pace_s)),
            );
            ui.colored_label(
                Color32::from_rgb(16, 185, 129),
                format!("起程: {}/km", fmt_pace_val(preview.min_pace_s)),
            );
            ui.colored_label(
                Color32::from_rgb(245, 158, 11),
                format!("尾程: {}/km", fmt_pace_val(preview.max_pace_s)),
            );
        });

        ui.add_space(2.0);

        // 折线图主画布
        let canvas_h = 160.0f32;
        let (response, painter) = ui.allocate_painter(
            Vec2::new(ui.available_width().max(280.0), canvas_h),
            egui::Sense::hover(),
        );
        let canvas = response.rect;

        // 边距：左侧放 Y 轴刻度文字，底部放时间/公里标签
        let pad_left = 54.0f32;
        let pad_right = 16.0f32;
        let pad_top = 14.0f32;
        let pad_bottom = 22.0f32;

        let plot_rect = Rect::from_min_max(
            Pos2::new(canvas.left() + pad_left, canvas.top() + pad_top),
            Pos2::new(canvas.right() - pad_right, canvas.bottom() - pad_bottom),
        );

        // 背景微底色
        painter.rect_filled(
            canvas,
            egui::Rounding::same(4.0),
            Color32::from_rgb(248, 250, 252),
        );
        painter.rect_stroke(
            canvas,
            egui::Rounding::same(4.0),
            Stroke::new(1.0_f32, Color32::from_rgb(226, 232, 240)),
        );

        // Y 轴范围：倒置轴！上方为较快（数值较小，如 4'00"），下方为较慢（数值较大，如 10'00"）
        let y_fast = (preview.min_pace_s - 25.0).clamp(240.0, 500.0).floor() / 30.0 * 30.0;
        let y_slow = (preview.max_pace_s + 25.0).clamp(360.0, 600.0).ceil() / 30.0 * 30.0;
        let y_span = (y_slow - y_fast).max(60.0);

        let to_plot_y = |p_sec: f64| -> f32 {
            let ratio = ((p_sec - y_fast) / y_span).clamp(0.0, 1.0) as f32;
            plot_rect.top() + ratio * plot_rect.height()
        };

        let dur_f = preview.total_dur_s.max(1) as f32;
        let to_plot_x = |t_sec: f64| -> f32 {
            let ratio = (t_sec as f32 / dur_f).clamp(0.0, 1.0);
            plot_rect.left() + ratio * plot_rect.width()
        };

        // 绘制 Y 轴水平辅助刻度线（整分钟线，如 5'00"、6'00"、7'00"、8'00"）
        let mut cur_y_val = (y_fast / 60.0).ceil() as i64 * 60;
        while (cur_y_val as f64) <= y_slow {
            let yy = to_plot_y(cur_y_val as f64);
            painter.line_segment(
                [
                    Pos2::new(plot_rect.left(), yy),
                    Pos2::new(plot_rect.right(), yy),
                ],
                Stroke::new(1.0_f32, Color32::from_rgb(235, 238, 242)),
            );
            // 刻度文字
            painter.text(
                Pos2::new(plot_rect.left() - 6.0, yy),
                egui::Align2::RIGHT_CENTER,
                format!("{}'00\"", cur_y_val / 60),
                egui::FontId::proportional(11.0),
                Color32::from_rgb(148, 163, 184),
            );
            cur_y_val += 60;
        }

        // 绘制平均配速金色基准线
        let avg_y = to_plot_y(preview.avg_pace_s);
        painter.line_segment(
            [
                Pos2::new(plot_rect.left(), avg_y),
                Pos2::new(plot_rect.right(), avg_y),
            ],
            Stroke::new(1.2_f32, Color32::from_rgb(234, 179, 8)),
        );
        painter.text(
            Pos2::new(plot_rect.right() - 2.0, avg_y - 8.0),
            egui::Align2::RIGHT_BOTTOM,
            format!("均速 {}", fmt_pace_val(preview.avg_pace_s)),
            egui::FontId::proportional(10.5),
            Color32::from_rgb(180, 83, 9),
        );

        // 绘制公里竖向虚线标记
        let total_km = preview.total_dis_m / 1000.0;
        for km_i in 1..=(total_km.floor() as i64) {
            let km_target = km_i as f64 * 1000.0;
            // 找到跨过该公里位置的时间
            if let Some(w) = preview.windows.iter().find(|w| w.cum_dist_m >= km_target) {
                let xx = to_plot_x(w.time_s);
                painter.line_segment(
                    [
                        Pos2::new(xx, plot_rect.top()),
                        Pos2::new(xx, plot_rect.bottom()),
                    ],
                    Stroke::new(1.0_f32, Color32::from_rgb(203, 213, 225)),
                );
                painter.text(
                    Pos2::new(xx, plot_rect.bottom() + 4.0),
                    egui::Align2::CENTER_TOP,
                    format!("{km_i}km"),
                    egui::FontId::proportional(10.5),
                    Color32::from_rgb(100, 116, 139),
                );
            }
        }

        // 绘制底部首尾时间刻度
        painter.text(
            Pos2::new(plot_rect.left(), plot_rect.bottom() + 4.0),
            egui::Align2::LEFT_TOP,
            "00:00",
            egui::FontId::proportional(10.5),
            Color32::from_rgb(148, 163, 184),
        );
        painter.text(
            Pos2::new(plot_rect.right(), plot_rect.bottom() + 4.0),
            egui::Align2::RIGHT_TOP,
            fmt_time_val(preview.total_dur_s),
            egui::FontId::proportional(10.5),
            Color32::from_rgb(148, 163, 184),
        );

        // 生成点列并绘制下方半透明填充多边形
        let pts: Vec<Pos2> = preview
            .windows
            .iter()
            .map(|w| Pos2::new(to_plot_x(w.time_s), to_plot_y(w.pace_s_per_km)))
            .collect();

        if pts.len() >= 2 {
            let mut fill_polygon = Vec::with_capacity(pts.len() + 2);
            fill_polygon.push(Pos2::new(pts[0].x, plot_rect.bottom()));
            fill_polygon.extend(pts.iter().copied());
            fill_polygon.push(Pos2::new(pts.last().unwrap().x, plot_rect.bottom()));

            let fill_mesh = egui::epaint::Mesh {
                indices: (1..fill_polygon.len() as u32 - 1)
                    .flat_map(|i| vec![0, i, i + 1])
                    .collect(),
                vertices: fill_polygon
                    .iter()
                    .map(|&p| egui::epaint::Vertex {
                        pos: p,
                        uv: egui::epaint::WHITE_UV,
                        color: Color32::from_rgba_unmultiplied(16, 185, 129, 30),
                    })
                    .collect(),
                texture_id: egui::TextureId::default(),
            };
            painter.add(fill_mesh);

            // 绘制主配速连续平滑折线
            painter.add(egui::Shape::line(
                pts.clone(),
                Stroke::new(2.2_f32, Color32::from_rgb(16, 185, 129)),
            ));
        }

        // 交互光标悬停拾取
        if let Some(hover_pos) = response.hover_pos() {
            if plot_rect.contains(hover_pos) {
                let hover_ratio = ((hover_pos.x - plot_rect.left()) / plot_rect.width()).clamp(0.0, 1.0);
                let hover_t = hover_ratio as f64 * preview.total_dur_s as f64;

                // 寻找离光标最近的时间窗口
                if let Some(best_w) = preview
                    .windows
                    .iter()
                    .min_by(|a, b| (a.time_s - hover_t).abs().partial_cmp(&(b.time_s - hover_t).abs()).unwrap())
                {
                    let h_x = to_plot_x(best_w.time_s);
                    let h_y = to_plot_y(best_w.pace_s_per_km);

                    // 竖向导引线
                    painter.line_segment(
                        [
                            Pos2::new(h_x, plot_rect.top()),
                            Pos2::new(h_x, plot_rect.bottom()),
                        ],
                        Stroke::new(1.0_f32, Color32::from_rgb(71, 85, 105)),
                    );

                    // 当前点光标圆点
                    painter.circle_filled(Pos2::new(h_x, h_y), 4.5, Color32::from_rgb(16, 185, 129));
                    painter.circle_stroke(
                        Pos2::new(h_x, h_y),
                        4.5,
                        Stroke::new(1.5_f32, Color32::WHITE),
                    );

                    // 悬浮 HUD 标签信息卡
                    let hud_text = format!(
                        "⏱ {} · 📏 {:.2} km · ⚡ {}/km (10s: {:.1}m) · 🏃 {:.0} spm · 步幅 {:.0} cm",
                        fmt_time_val(best_w.time_s.round() as i64),
                        best_w.cum_dist_m / 1000.0,
                        fmt_pace_val(best_w.pace_s_per_km),
                        best_w.window_dist_m,
                        best_w.cadence,
                        best_w.stride_cm
                    );

                    let hud_x = if h_x > plot_rect.center().x {
                        h_x - 12.0
                    } else {
                        h_x + 12.0
                    };
                    let align = if h_x > plot_rect.center().x {
                        egui::Align2::RIGHT_TOP
                    } else {
                        egui::Align2::LEFT_TOP
                    };

                    painter.text(
                        Pos2::new(hud_x, plot_rect.top() + 6.0),
                        align,
                        hud_text,
                        egui::FontId::proportional(11.5),
                        Color32::from_rgb(15, 23, 42),
                    );
                }
            }
        }

        // 底部每公里分段统计小卡片（徽章流式排列）
        ui.add_space(2.0);
        ui.horizontal_wrapped(|ui| {
            ui.colored_label(theme::plain(), "公里分段：");
            for lap in &preview.laps {
                let badge_text = if lap.is_full {
                    format!("第 {} 公里: {}", lap.lap_num, fmt_pace_val(lap.pace_s_per_km))
                } else {
                    format!(
                        "余 {:.0}m: {}",
                        lap.dist_m,
                        fmt_pace_val(lap.pace_s_per_km)
                    )
                };
                let tooltip = format!(
                    "用时: {} · 步频: {:.0} spm · 步幅: {:.0} cm",
                    fmt_time_val(lap.dur_s),
                    lap.cadence,
                    lap.stride_cm
                );
                ui.label(
                    egui::RichText::new(badge_text)
                        .size(11.5)
                        .color(Color32::from_rgb(30, 41, 59)),
                )
                .on_hover_text(tooltip);
                ui.add_space(4.0);
            }
        });
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_pace_preview_windows_and_splits() {
        let p = build_pace_preview(2350.0, 1020, 42, false);
        assert!(!p.windows.is_empty());
        assert!(!p.laps.is_empty());
        assert_eq!(p.laps.len(), 3); // 1km, 2km, 350m
        assert!(p.avg_pace_s >= 240.0 && p.avg_pace_s <= 600.0);
        assert!(p.min_pace_s >= 240.0);
        assert!(p.max_pace_s <= 600.0);
        assert!(p.min_pace_s <= p.max_pace_s);

        // 验证首公里比末公里快（自然体能前快后慢）
        assert!(p.laps[0].pace_s_per_km < p.laps[1].pace_s_per_km);

        // 验证最后一个残余时间窗（1020s % 10 = 0，换一个非整10秒测试末尾无异常冲刺，且绝无 1800 步频暴飙）
        let p_odd = build_pace_preview(2350.0, 1023, 42, false);
        let last_w = p_odd.windows.last().unwrap();
        let prev_w = &p_odd.windows[p_odd.windows.len() - 2];
        // 最后一个窗口配速必须连续（与倒数第二窗配速差异小于 15秒/km），绝不突然暴飙到 4'00"
        assert!(
            (last_w.pace_s_per_km - prev_w.pace_s_per_km).abs() < 15.0,
            "末端残余窗口发生突兀加速: prev={} last={}",
            last_w.pace_s_per_km, prev_w.pace_s_per_km
        );
        // 验证末端残余窗口步频绝不暴飙到 1800（必须严格在 165~205 spm 范围内）
        assert!(
            last_w.cadence >= 165.0 && last_w.cadence <= 205.0,
            "末端残余窗口步频异常暴飙: cadence={}",
            last_w.cadence
        );

        // 验证步频具有生动的真人随机起伏性（非死板直线，极差在 8~30 spm 之间）
        let min_cad = p.windows.iter().map(|w| w.cadence).fold(f64::INFINITY, f64::min);
        let max_cad = p.windows.iter().map(|w| w.cadence).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_cad - min_cad >= 8.0,
            "步频缺乏真人起伏感（过于平直）: min={} max={}",
            min_cad, max_cad
        );

        // 验证开启中途喘息降速：中段配速有自然的平缓波谷（降速槽），且绝不越出 10'00" 标红
        let p_slow = build_pace_preview(2350.0, 1020, 42, true);
        assert!(p_slow.max_pace_s <= 600.0, "中途降速绝不标红超标");
        assert!(p_slow.max_pace_s > p.max_pace_s, "开启中途缓行后应有更深入的缓跑喘息段");
    }
}
