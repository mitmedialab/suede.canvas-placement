//! Turtle geometry: commands -> pen-down strokes, rotation, body sweep.

use crate::types::{Command, Point, RobotBody, Stroke};

/// Arcs are flattened into chords of at most this many degrees.
pub const ARC_STEP_DEG: f64 = 6.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub heading_deg: f64,
}

/// Turtle-integrate `commands` (lengths multiplied by `scale`) from `start`.
/// Mirrors canvas.py `commands_to_strokes_with_pose` applied to robots.py's
/// `scale_commands` output.
pub fn commands_to_strokes(commands: &[Command], scale: f64, start: Pose) -> (Vec<Stroke>, Pose) {
    let (mut x, mut y, mut heading) = (start.x, start.y, start.heading_deg);
    let mut strokes: Vec<Stroke> = Vec::new();
    let mut current: Stroke = Vec::new();

    fn flush(strokes: &mut Vec<Stroke>, current: &mut Stroke) {
        if current.len() >= 2 {
            strokes.push(std::mem::take(current));
        } else {
            current.clear();
        }
    }

    for cmd in commands {
        match *cmd {
            Command::Line { distance, pen_down } => {
                let d = distance * scale;
                let rad = heading.to_radians();
                let (nx, ny) = (x + d * rad.cos(), y + d * rad.sin());
                if pen_down {
                    if current.is_empty() {
                        current.push((x, y));
                    }
                    current.push((nx, ny));
                } else {
                    flush(&mut strokes, &mut current);
                }
                x = nx;
                y = ny;
            }
            Command::Spin { degrees } => heading += degrees,
            Command::Arc { radius, degrees } => {
                let steps = ((degrees.abs() / ARC_STEP_DEG).ceil() as i64).max(1);
                let dtheta = degrees / steps as f64;
                let seg_len = (radius * scale) * dtheta.abs().to_radians();
                for _ in 0..steps {
                    let rad = heading.to_radians();
                    let (nx, ny) = (x + seg_len * rad.cos(), y + seg_len * rad.sin());
                    if current.is_empty() {
                        current.push((x, y));
                    }
                    current.push((nx, ny));
                    x = nx;
                    y = ny;
                    heading += dtheta;
                }
            }
        }
    }
    flush(&mut strokes, &mut current);
    (strokes, Pose { x, y, heading_deg: heading })
}

/// The job's ink at `scale`, first ink at the origin, rotated by `heading0`
/// (robots.py `replay_to_world(scale_commands(...), 0, 0, heading0)`).
pub fn local_strokes(commands: &[Command], scale: f64, heading0: f64) -> Vec<Stroke> {
    let (strokes, _) = commands_to_strokes(commands, scale, Pose { x: 0.0, y: 0.0, heading_deg: 0.0 });
    let rad = heading0.to_radians();
    let (c, s) = (rad.cos(), rad.sin());
    strokes
        .into_iter()
        .map(|st| st.into_iter().map(|(px, py)| (px * c - py * s, px * s + py * c)).collect())
        .collect()
}

/// Rotate about the origin (canvas.py `rotate_strokes`).
pub fn rotate_strokes(strokes: &[Stroke], degrees: f64) -> Vec<Stroke> {
    if degrees == 0.0 {
        return strokes.to_vec();
    }
    let rad = degrees.to_radians();
    let (c, s) = (rad.cos(), rad.sin());
    strokes
        .iter()
        .map(|st| st.iter().map(|&(px, py)| (px * c - py * s, px * s + py * c)).collect())
        .collect()
}

/// The rectangle a robot's body sweeps while its pen goes from `a` to `b`,
/// inflated by `margin` on every side (canvas.py `_body_sweep`). `None` for a
/// zero-length segment.
pub fn body_sweep(a: Point, b: Point, body: &RobotBody, margin: f64) -> Option<[Point; 4]> {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let dist = dx.hypot(dy);
    if dist == 0.0 {
        return None;
    }
    let (ux, uy) = (dx / dist, dy / dist);
    let (nx, ny) = (-uy, ux);
    let back = body.pen_from_tail_mm() + margin;
    let front = body.pen_from_nose_mm + margin;
    let (rx, ry) = (a.0 - ux * back, a.1 - uy * back);
    let (fx, fy) = (b.0 + ux * front, b.1 + uy * front);
    let hw = body.width_mm / 2.0 + margin;
    Some([
        (rx + nx * hw, ry + ny * hw),
        (fx + nx * hw, fy + ny * hw),
        (fx - nx * hw, fy - ny * hw),
        (rx - nx * hw, ry - ny * hw),
    ])
}

/// (min_x, min_y, max_x, max_y) over all points, or None if there are none.
pub fn bounds(strokes: &[Stroke]) -> Option<(f64, f64, f64, f64)> {
    let mut it = strokes.iter().flatten();
    let &(x0, y0) = it.next()?;
    Some(it.fold((x0, y0, x0, y0), |(a, b, c, d), &(x, y)| (a.min(x), b.min(y), c.max(x), d.max(y))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_and_spin() {
        let cmds = vec![
            Command::Line { distance: 10.0, pen_down: true },
            Command::Spin { degrees: 90.0 },
            Command::Line { distance: 5.0, pen_down: true },
            Command::Line { distance: 5.0, pen_down: false },
            Command::Line { distance: 1.0, pen_down: true },
        ];
        let (s, end) = commands_to_strokes(&cmds, 2.0, Pose { x: 0.0, y: 0.0, heading_deg: 0.0 });
        assert_eq!(s.len(), 2);
        assert_eq!(s[0].len(), 3);
        assert!((s[0][1].0 - 20.0).abs() < 1e-9);
        assert!((s[0][2].1 - 10.0).abs() < 1e-9);
        assert!((end.y - 22.0).abs() < 1e-9);
    }

    #[test]
    fn arc_is_flattened_in_6_degree_steps() {
        let cmds = vec![Command::Arc { radius: 10.0, degrees: 90.0 }];
        let (s, end) = commands_to_strokes(&cmds, 1.0, Pose { x: 0.0, y: 0.0, heading_deg: 0.0 });
        assert_eq!(s[0].len(), 16);
        assert!((end.heading_deg - 90.0).abs() < 1e-9);
    }
}
