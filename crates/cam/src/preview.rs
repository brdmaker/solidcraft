//! A top view of the sheet with its toolpaths: the sheet and its margin, every placed part's
//! outline, each operation's cutting paths in the colour of its kind, rapid moves as thin grey
//! lines and drilling cycles as rings with a cross. As a PNG (`tiny-skia`, no text: the colours
//! are in [`legend`]) or an SVG with a legend.

use std::fmt::Write as _;

use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, Stroke as SkStroke, StrokeDash, Transform};

use crate::region::Area;
use crate::toolpath::{Move, OpKind, Operation, StrokeKind, trace};
use crate::{CamError, Result};

/// The most pixels a preview may have on its longer side.
pub const MAX_PIXELS: u32 = 8192;

/// What to draw, in sheet coordinates (mm, X right, Y back, Z0 the sheet's top).
pub struct Scene<'a> {
    pub sheet: [f64; 2],
    pub margin: f64,
    /// Each placed part's outline (turned and moved onto the sheet).
    pub outlines: &'a [Area],
    pub ops: &'a [Operation],
    /// Where the tool is before the first operation.
    pub clearance: f64,
}

/// The colour of an operation's paths, as `#rrggbb`.
pub fn colour(kind: OpKind) -> &'static str {
    match kind {
        OpKind::Facing => "#8fb8de",
        OpKind::Countersink => "#00897b",
        OpKind::Roughing => "#1f6fd1",
        OpKind::Drilling => "#d62728",
        OpKind::HoleMilling => "#ef8a17",
        OpKind::FloorFinish => "#2ca02c",
        OpKind::WallFinish => "#13784a",
        OpKind::Chamfer => "#9b59b6",
        OpKind::Profile => "#222222",
        OpKind::EdgeChamfer => "#c2185b",
        OpKind::ThroughHole => "#bf360c",
        OpKind::Separation => "#6d4c41",
    }
}

/// Every kind with its colour and name, in machining order.
pub fn legend() -> Vec<(OpKind, &'static str, &'static str)> {
    [
        (OpKind::Facing, "facing"),
        (OpKind::Countersink, "countersinks (end mill)"),
        (OpKind::Roughing, "roughing"),
        (OpKind::HoleMilling, "counterbores, recesses"),
        (OpKind::FloorFinish, "floors"),
        (OpKind::WallFinish, "walls"),
        (OpKind::Chamfer, "countersinks, chamfers"),
        (OpKind::Profile, "profile"),
        (OpKind::EdgeChamfer, "edge chamfer"),
        (OpKind::Drilling, "drilling"),
        (OpKind::ThroughHole, "through holes"),
        (OpKind::Separation, "separation"),
    ]
    .into_iter()
    .map(|(k, n)| (k, colour(k), n))
    .collect()
}

fn rgb(hex: &str, alpha: f32) -> Color {
    let v = |i: usize| hex.get(i..i + 2).and_then(|s| u8::from_str_radix(s, 16).ok()).unwrap_or(0);
    let mut c = Color::from_rgba8(v(1), v(3), v(5), 255);
    c.apply_opacity(alpha);
    c
}

/// Arc steps of the drawing: 3°.
const ARC_STEP: f64 = 3.0 * std::f64::consts::PI / 180.0;

/// The scene as a PNG `width` pixels wide (the height follows the sheet), with a margin of
/// 4 % round the sheet.
pub fn png(scene: &Scene, width: u32) -> Result<Vec<u8>> {
    let (w, h) = (scene.sheet[0], scene.sheet[1]);
    if !(w > 0.0 && h > 0.0 && w.is_finite() && h.is_finite()) {
        return Err(CamError::Invalid("the sheet has no size".into()));
    }
    let width = width.clamp(64, MAX_PIXELS);
    let pad = 0.04 * w.max(h);
    let scale = width as f64 / (w + 2.0 * pad);
    let height = (((h + 2.0 * pad) * scale).round() as u32).clamp(16, MAX_PIXELS);
    let mut pm = Pixmap::new(width, height).ok_or_else(|| CamError::Invalid("preview size".into()))?;
    pm.fill(rgb("#f4f5f7", 1.0));
    // mm → pixels, Y up
    let tf = Transform::from_row(scale as f32, 0.0, 0.0, -(scale as f32), (pad * scale) as f32, ((h + pad) * scale) as f32);
    let px = |mm: f64| (mm * scale) as f32;
    let paint = |hex: &str, a: f32| {
        let mut p = Paint::default();
        p.set_color(rgb(hex, a));
        p.anti_alias = true;
        p
    };
    let line = |width_px: f32| SkStroke { width: width_px / scale as f32, ..SkStroke::default() };

    // the sheet and its margin
    if let Some(rect) = tiny_skia::Rect::from_xywh(0.0, 0.0, w as f32, h as f32) {
        let sheet = PathBuilder::from_rect(rect);
        pm.fill_path(&sheet, &paint("#dfe3e8", 1.0), FillRule::Winding, tf, None);
        pm.stroke_path(&sheet, &paint("#8a939c", 1.0), &line(1.5), tf, None);
    }
    let m = scene.margin;
    if m > 0.0
        && let Some(rect) = tiny_skia::Rect::from_xywh(m as f32, m as f32, (w - 2.0 * m) as f32, (h - 2.0 * m) as f32)
    {
        let mut dashed = line(1.0);
        dashed.dash = StrokeDash::new(vec![6.0 / scale as f32, 4.0 / scale as f32], 0.0);
        pm.stroke_path(&PathBuilder::from_rect(rect), &paint("#a7b0b9", 1.0), &dashed, tf, None);
    }
    // the parts
    for a in scene.outlines {
        let mut pb = PathBuilder::new();
        for ring in a.rings() {
            for (i, p) in ring.iter().enumerate() {
                if i == 0 {
                    pb.move_to(p[0] as f32, p[1] as f32);
                } else {
                    pb.line_to(p[0] as f32, p[1] as f32);
                }
            }
            pb.close();
        }
        if let Some(path) = pb.finish() {
            pm.fill_path(&path, &paint("#ffffff", 1.0), FillRule::EvenOdd, tf, None);
            pm.stroke_path(&path, &paint("#5b6570", 1.0), &line(1.0), tf, None);
        }
    }
    // the toolpaths: rapids first, underneath
    let mut at = [f64::NAN, f64::NAN, scene.clearance];
    let mut traced = Vec::new();
    for op in scene.ops {
        traced.push((op.kind, trace(op, at, ARC_STEP)));
        if let Some(end) = op.moves.last().map(Move::end) {
            at = end;
        }
    }
    let path_of = |pts: &[[f64; 3]]| {
        let mut pb = PathBuilder::new();
        for (i, p) in pts.iter().enumerate() {
            if i == 0 {
                pb.move_to(p[0] as f32, p[1] as f32);
            } else {
                pb.line_to(p[0] as f32, p[1] as f32);
            }
        }
        pb.finish()
    };
    for (_, strokes) in &traced {
        for s in strokes.iter().filter(|s| s.kind == StrokeKind::Rapid) {
            if let Some(path) = path_of(&s.pts) {
                pm.stroke_path(&path, &paint("#9aa3ad", 0.6), &line(0.6), tf, None);
            }
        }
    }
    for (kind, strokes) in &traced {
        let width_px = if *kind == OpKind::Profile { 1.6 } else { 1.0 };
        for s in strokes.iter().filter(|s| s.kind == StrokeKind::Cut) {
            if let Some(path) = path_of(&s.pts) {
                pm.stroke_path(&path, &paint(colour(*kind), 0.9), &line(width_px), tf, None);
            }
        }
    }
    // drilling cycles: a ring and a cross
    for op in scene.ops {
        for m in &op.moves {
            if let Move::Drill { at, .. } = m {
                let r = 4.0 / scale as f32;
                let (x, y) = (at[0] as f32, at[1] as f32);
                let mut pb = PathBuilder::new();
                pb.push_circle(x, y, r);
                pb.move_to(x - r * 1.4, y);
                pb.line_to(x + r * 1.4, y);
                pb.move_to(x, y - r * 1.4);
                pb.line_to(x, y + r * 1.4);
                if let Some(path) = pb.finish() {
                    pm.stroke_path(&path, &paint(colour(OpKind::Drilling), 1.0), &line(1.4), tf, None);
                }
            }
        }
    }
    // legend swatches along the bottom edge, in machining order (names in `legend()`)
    let used: Vec<OpKind> = legend().into_iter().map(|(k, ..)| k).filter(|k| scene.ops.iter().any(|o| o.kind == *k)).collect();
    for (i, k) in used.iter().enumerate() {
        let x = px(pad) + i as f32 * 26.0;
        if let Some(rect) = tiny_skia::Rect::from_xywh(x, height as f32 - px(pad) * 0.7, 18.0, 8.0) {
            pm.fill_rect(rect, &paint(colour(*k), 1.0), Transform::identity(), None);
        }
    }
    encode(&pm)
}

fn encode(pm: &Pixmap) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut out, pm.width(), pm.height());
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut w = enc.write_header().map_err(|e| CamError::Invalid(format!("png: {e}")))?;
        // opaque throughout (the background is), so premultiplied equals straight alpha
        w.write_image_data(pm.data()).map_err(|e| CamError::Invalid(format!("png: {e}")))?;
    }
    Ok(out)
}

/// The scene as SVG (millimetres, Y up), with a legend naming each kind of operation drawn.
pub fn svg(scene: &Scene) -> String {
    let (w, h) = (scene.sheet[0], scene.sheet[1]);
    let pad = 0.04 * w.max(h);
    let legend_h = 0.06 * h.max(60.0);
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{:.3} {:.3} {:.3} {:.3}" width="{:.0}mm" height="{:.0}mm">"#,
        -pad,
        -pad,
        w + 2.0 * pad,
        h + 2.0 * pad + legend_h,
        w + 2.0 * pad,
        h + 2.0 * pad + legend_h
    );
    let _ = write!(
        s,
        r##"<rect x="{:.3}" y="{:.3}" width="{:.3}" height="{:.3}" fill="#f4f5f7"/>"##,
        -pad,
        -pad,
        w + 2.0 * pad,
        h + 2.0 * pad + legend_h
    );
    // drawing in sheet coordinates: flip Y
    let _ = write!(s, r#"<g transform="translate(0 {h:.3}) scale(1 -1)" fill="none" stroke-linejoin="round" stroke-linecap="round">"#);
    let _ = write!(s, r##"<rect width="{w:.3}" height="{h:.3}" fill="#dfe3e8" stroke="#8a939c" stroke-width="0.6"/>"##);
    let m = scene.margin;
    if m > 0.0 {
        let _ = write!(
            s,
            r##"<rect x="{m:.3}" y="{m:.3}" width="{:.3}" height="{:.3}" stroke="#a7b0b9" stroke-width="0.3" stroke-dasharray="3 2"/>"##,
            w - 2.0 * m,
            h - 2.0 * m
        );
    }
    for a in scene.outlines {
        let mut d = String::new();
        for ring in a.rings() {
            for (i, p) in ring.iter().enumerate() {
                let _ = write!(d, "{}{:.3} {:.3}", if i == 0 { "M" } else { "L" }, p[0], p[1]);
            }
            d.push('Z');
        }
        let _ = write!(s, r##"<path d="{d}" fill="#ffffff" fill-rule="evenodd" stroke="#5b6570" stroke-width="0.4"/>"##);
    }
    let mut at = [f64::NAN, f64::NAN, scene.clearance];
    let mut cuts = String::new();
    let mut rapids = String::new();
    for op in scene.ops {
        let mut d = String::new();
        for st in trace(op, at, ARC_STEP) {
            let target = if st.kind == StrokeKind::Rapid { &mut rapids } else { &mut d };
            for (i, p) in st.pts.iter().enumerate() {
                let _ = write!(target, "{}{:.3} {:.3}", if i == 0 { "M" } else { "L" }, p[0], p[1]);
            }
        }
        if let Some(end) = op.moves.last().map(Move::end) {
            at = end;
        }
        let width = if op.kind == OpKind::Profile { 0.5 } else { 0.3 };
        let _ = write!(
            cuts,
            r#"<path d="{d}" stroke="{}" stroke-width="{width}" stroke-opacity="0.9"><title>{}</title></path>"#,
            colour(op.kind),
            xml(&op.name)
        );
        for m in &op.moves {
            if let Move::Drill { at, bottom, .. } = m {
                let _ = write!(
                    cuts,
                    r#"<g stroke="{}" stroke-width="0.4"><circle cx="{:.3}" cy="{:.3}" r="1.5"/><path d="M{:.3} {:.3}h4M{:.3} {:.3}v4"/><title>{} to Z{:.3}</title></g>"#,
                    colour(OpKind::Drilling),
                    at[0],
                    at[1],
                    at[0] - 2.0,
                    at[1],
                    at[0],
                    at[1] - 2.0,
                    xml(&op.name),
                    bottom
                );
            }
        }
    }
    let _ = write!(s, r##"<path d="{rapids}" stroke="#9aa3ad" stroke-width="0.2" stroke-opacity="0.6"/>"##);
    s.push_str(&cuts);
    s.push_str("</g>");
    // legend, under the sheet
    let size = legend_h * 0.35;
    let mut x = 0.0;
    for (k, c, name) in legend() {
        if !scene.ops.iter().any(|o| o.kind == k) {
            continue;
        }
        let y = h + pad + legend_h * 0.3;
        let _ = write!(s, r#"<rect x="{x:.3}" y="{y:.3}" width="{size:.3}" height="{size:.3}" fill="{c}"/>"#);
        let _ = write!(
            s,
            r##"<text x="{:.3}" y="{:.3}" font-family="sans-serif" font-size="{:.3}" fill="#333">{name}</text>"##,
            x + size * 1.3,
            y + size * 0.85,
            size
        );
        x += size * 1.6 + size * 0.55 * name.len() as f64;
    }
    s.push_str("</svg>");
    s
}

fn xml(t: &str) -> String {
    t.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}
