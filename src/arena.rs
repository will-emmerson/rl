//! The arena: a rounded box, written as a signed-distance field.
//!
//! Dimensions come from RocketSim's `RLConst.h` (`ARENA_EXTENT_X/Y`,
//! `ARENA_HEIGHT`), which are in Unreal units (1 uu = 1 cm). Real Rocket League
//! collides against a *mesh* (dumped with RLArenaCollisionDumper) whose corners
//! are chamfered and whose wall-to-ceiling join is rounded. A plain box gets the
//! extents right but its perfectly sharp corners trap the car, and its floor and
//! ceiling meet the walls at 90° so a car cannot drive from a wall onto the
//! ceiling. This is a box with those joins rounded.
//!
//! The field is the `max` of three rounded shapes, so
//!
//! * the vertical wall corners are chamfered with radius `CORNER_RADIUS`,
//! * the wall-to-ceiling join is filleted with the same radius (which is what
//!   lets a car climb a wall and roll onto the ceiling), and
//! * the wall-to-floor join is filleted with the smaller `FLOOR_FILLET` so the
//!   car has a smooth surface to start the climb from.
//!
//! The floor and ceiling themselves stay flat. The radii are gameplay guesses,
//! not Rocket League figures: they are not in `RLConst.h`, and RL's collision
//! hull is not a rounded box.
//!
//! That rounded box is the *shell*: `shell_inside_distance` / `shell_normal_at`.
//! The physics field (`inside_distance` / `normal_at`) additionally opens a goal
//! mouth in each end wall and adds a curved net scoop behind it (see
//! `goal_field`). The scoop lets the ball cross the line and score, and lets a car
//! drive in low and ride the curve out at the top. The viewer draws the shell
//! surface, cuts the same openings out of it, and adds the goal frame and scoop.

use crate::math::Vec3;

/// Radius of the vertical corner chamfer and the wall-to-ceiling fillet, in
/// metres.
pub const CORNER_RADIUS: f64 = 4.0;
/// Radius of the wall-to-floor fillet, in metres. Deliberately much smaller
/// than `CORNER_RADIUS`: a big fillet here would eat into the playable floor
/// near every wall.
pub const FLOOR_FILLET: f64 = 1.0;

// Soccar goal geometry, in metres. Unlike the fillet radii above these are not
// guesses: RocketSim bakes the goal into its dumped arena collision mesh rather
// than naming it in `RLConst.h`, so the shape comes from the page RocketSim's
// own comments cite, the RLBot "Useful Game Values" wiki. Unreal units are
// 1 cm, so the conversions are exact divisions by 100.

/// Half the goal opening. RLBot `goal center-to-post = 892.755 uu`, so the mouth
/// is 1785.51 uu (17.8551 m) wide, split either side of the centre line.
pub const GOAL_HALF_WIDTH: f64 = 8.927_55;
/// RLBot `goal height = 642.775 uu`: the underside of the crossbar.
pub const GOAL_HEIGHT: f64 = 6.427_75;
/// RLBot `goal depth = 880 uu`: how far the net sits behind the goal line.
pub const GOAL_DEPTH: f64 = 8.80;

// The goal is not a box. RocketSim bakes the goal into its dumped soccar
// collision mesh; parsing that mesh shows the net is a **semicircular scoop**
// swept across the goal's width, with the ball/car cavity being the rectangle
// (mouth -> `GOAL_NET_CENTER_DEPTH`) united with that scoop. Fitting a circle to
// the mesh's cross-section gives the three figures below, in metres:
//
//   radius 2.70, centre 6.11 deep and 2.59 high (so the scoop is tangent to the
//   floor at ~5.4 m across and bulges back to the full `GOAL_DEPTH` at ~2.6 m up).
//
// This curvature is what lets a car drive in at the bottom of the mouth, ride the
// curve up the back, and come out at the top - a sharp box does not.

/// Radius of the net scoop, in metres (RocketSim soccar mesh: ~270 uu).
pub const GOAL_NET_RADIUS: f64 = 2.70;
/// How deep the scoop's centre sits behind the goal line, in metres (~611 uu).
pub const GOAL_NET_CENTER_DEPTH: f64 = 6.11;
/// How high the scoop's centre sits, in metres (~259 uu).
pub const GOAL_NET_CENTER_HEIGHT: f64 = 2.59;
/// RocketSim `SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y = 5124.25 uu`: the base goal
/// line, a little past `ARENA_EXTENT_Y = 5120`. RocketSim scores when the ball's
/// *centre* passes `base + ballRadius`, i.e. once the ball has fully crossed the
/// line - see [`Arena::ball_in_goal`].
pub const GOAL_SCORE_BASE_Z: f64 = 51.2425;

/// Radius of the fillet rounding the goal recess's concave corners, in metres.
/// The recess is the mouth rectangle united with the net scoop, which leaves a
/// sharp corner where the scoop's top meets the flat back wall and where the back
/// wall meets the roof. A car is a point contact held `CAR_HALF_HEIGHT` off the
/// surface, so it follows smooth joins (the arena's own corners are filleted too)
/// but jams on a sharp one; rounding them `GOAL_FILLET` lets a car ride the net up
/// and out. Not a RocketSim figure - a gameplay fix, like `CORNER_RADIUS`.
pub const GOAL_FILLET: f64 = 0.8;

/// Which end a goal is at. The `-Z` goal is blue's (Rocket League's blue goal is
/// at `-Y`, the direction the car spawns facing); `+Z` is orange's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Goal {
    Minus,
    Plus,
}

#[derive(Clone, Copy, Debug)]
pub struct Arena {
    pub half_x: f64,
    pub half_z: f64,
    pub height: f64,
}

impl Default for Arena {
    fn default() -> Self {
        // Standard "Soccar" arena, converted from uu to metres:
        //   ARENA_EXTENT_X = 4096 uu half-width   -> 40.96 m
        //   ARENA_EXTENT_Y = 5120 uu half-length  -> 51.20 m
        //   ARENA_HEIGHT   = 2048 uu              -> 20.48 m
        // So the field is 81.92 m across, 102.40 m long and 20.48 m high. The
        // long axis is Z here (Rocket League's Y), which is the direction the
        // car spawns facing.
        Self {
            half_x: 40.96,
            half_z: 51.20,
            height: 20.48,
        }
    }
}

impl Arena {
    /// Signed distance to the *arena shell*: the rounded box with **no goal
    /// mouths**. Negative inside, positive outside, zero exactly on a surface.
    /// This is the surface the viewer draws (it cuts the goal openings out of
    /// the shell separately, so the end walls stay solid here).
    pub fn shell_inside_distance(&self, p: Vec3) -> f64 {
        self.wall_distance(p)
            .max(self.floor_distance(p))
            .max(self.ceiling_distance(p))
    }

    /// Unit normal pointing *into* the playable space for the shell field. Over
    /// the rounded corners the field is smooth, so there is no face to choose
    /// between.
    pub fn shell_normal_at(&self, p: Vec3) -> Vec3 {
        let gradient = self.shell_gradient(p);
        let normal = -gradient;
        if normal.length_squared() > 1e-18 {
            normal.normalized()
        } else {
            Vec3::UP
        }
    }

    /// Outward gradient of [`Self::shell_inside_distance`].
    fn shell_gradient(&self, p: Vec3) -> Vec3 {
        let w = self.wall_distance(p);
        let f = self.floor_distance(p);
        let c = self.ceiling_distance(p);
        // The active constraint is the one whose field is greatest (closest to
        // the surface coming from inside); its negated gradient is the inward
        // normal.
        if w >= f && w >= c {
            self.wall_gradient(p)
        } else if f >= c {
            self.floor_gradient(p)
        } else {
            self.ceiling_gradient(p)
        }
    }

    /// Signed distance to the playable surface with the goals opened: the field
    /// the ball and car collide against. Same as [`Self::shell_inside_distance`]
    /// except each end wall is cut away inside its goal mouth and the goal recess
    /// is added behind it.
    pub fn inside_distance(&self, p: Vec3) -> f64 {
        self.field(p).0
    }

    /// Unit normal pointing *into* the playable space at `p`, i.e. the direction
    /// an object resting on the surface is pushed. Over the rounded corners the
    /// field is smooth, so there is no face to choose between.
    pub fn normal_at(&self, p: Vec3) -> Vec3 {
        let normal = -self.field(p).1;
        if normal.length_squared() > 1e-18 {
            normal.normalized()
        } else {
            Vec3::UP
        }
    }

    /// `(signed_distance, outward_gradient)` of the physics field.
    ///
    /// The arena is a rounded box; a goal is a rectangular recess cut into the
    /// middle of each end wall. Away from the ends the shell is used unchanged.
    /// Within the goal-mouth footprint there is no end wall, and behind the end
    /// walls only the recess exists. The mouth sits far from the rounded vertical
    /// corners and the wall-to-ceiling fillet, so the flat-face cases below are
    /// exact there.
    fn field(&self, p: Vec3) -> (f64, Vec3) {
        if p.z.abs() > self.half_z {
            // Behind an end wall: only the goal recess is playable (outside the
            // recess, and past the net, the field is positive - solid).
            return self.goal_field(p);
        }
        if self.in_goal_footprint(p) {
            // Inside the arena band, within the mouth footprint: the end wall is
            // gone here (the goal opening), so only the side walls, floor and
            // ceiling constrain - all flat this far from the rounded corners.
            let sx = if p.x >= 0.0 { 1.0 } else { -1.0 };
            return max3(
                (p.x.abs() - self.half_x, Vec3::new(sx, 0.0, 0.0)),
                (-p.y, Vec3::new(0.0, -1.0, 0.0)),
                (p.y - self.height, Vec3::new(0.0, 1.0, 0.0)),
            );
        }
        (self.shell_inside_distance(p), self.shell_gradient(p))
    }

    /// True if `p` is within the goal-mouth cross-section: between the posts and
    /// under the crossbar, at either end.
    fn in_goal_footprint(&self, p: Vec3) -> bool {
        p.x.abs() <= GOAL_HALF_WIDTH && (0.0..=GOAL_HEIGHT).contains(&p.y)
    }

    /// Field for the goal recess: the rectangular part (`|x| <= GOAL_HALF_WIDTH`,
    /// `0 <= y <= GOAL_HEIGHT`, from the goal line back to `GOAL_NET_CENTER_DEPTH`)
    /// united with the net scoop, a semicircle of `GOAL_NET_RADIUS` centred at
    /// depth `GOAL_NET_CENTER_DEPTH`, height `GOAL_NET_CENTER_HEIGHT` and swept
    /// across the width. The scoop is what makes the goal curved.
    fn goal_field(&self, p: Vec3) -> (f64, Vec3) {
        let az = p.z.abs();
        let sx = if p.x >= 0.0 { 1.0 } else { -1.0 };
        let sz = if p.z >= 0.0 { 1.0 } else { -1.0 };

        // The lateral walls of the recess (the posts / side net), shared by both
        // pieces.
        let qx = p.x.abs() - GOAL_HALF_WIDTH;
        let qx_grad = Vec3::new(sx, 0.0, 0.0);

        // Rectangular part: the net's flat back at `GOAL_NET_CENTER_DEPTH`, the
        // floor at y = 0 and the roof at GOAL_HEIGHT. The mouth face is open, so
        // it is deliberately not a constraint here. The back/roof corner is
        // filleted (`smax`) so a car can roll up the back onto the roof.
        let back = self.half_z + GOAL_NET_CENTER_DEPTH;
        let bz = az - back;
        let by = (-p.y).max(p.y - GOAL_HEIGHT);
        let by_grad = if -p.y >= p.y - GOAL_HEIGHT {
            Vec3::new(0.0, -1.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let rect = max2(
            (qx, qx_grad),
            smax((bz, Vec3::new(0.0, 0.0, sz)), (by, by_grad), GOAL_FILLET),
        );

        // Net scoop: distance to the circle, kept inside the lateral walls.
        let dz = az - back;
        let dy = p.y - GOAL_NET_CENTER_HEIGHT;
        let len = (dz * dz + dy * dy).sqrt();
        let rad = len - GOAL_NET_RADIUS;
        let rad_grad = if len > 1e-12 {
            Vec3::new(0.0, dy / len, sz * dz / len)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let scoop = max2((rad, rad_grad), (qx, qx_grad));

        // Playable region = rectangle ∪ scoop, with the scoop-to-back-wall seam
        // filleted (`smin`) so riding the curve up onto the back wall is smooth.
        smin(rect, scoop, GOAL_FILLET)
    }

    /// Rounded-rectangle distance in the x/z plane: the walls with their four
    /// vertical edges chamfered by `CORNER_RADIUS`. Extruded in y, this is the
    /// pillar the rest of the arena carves into.
    fn wall_distance(&self, p: Vec3) -> f64 {
        let r = CORNER_RADIUS;
        let qx = p.x.abs() - (self.half_x - r);
        let qz = p.z.abs() - (self.half_z - r);
        let ox = qx.max(0.0);
        let oz = qz.max(0.0);
        let outside = (ox * ox + oz * oz).sqrt();
        let inside = qx.max(qz).min(0.0);
        outside + inside - r
    }

    /// The floor at `y = 0`, with a `FLOOR_FILLET` radius where it meets the
    /// walls. Exact for a single convex constraint, i.e. -y over the flat floor.
    fn floor_distance(&self, p: Vec3) -> f64 {
        let r = FLOOR_FILLET;
        let qx = (p.x.abs() - (self.half_x - r)).max(0.0);
        let qz = (p.z.abs() - (self.half_z - r)).max(0.0);
        let qy = (r - p.y).max(0.0);
        (qx * qx + qy * qy + qz * qz).sqrt() - r
    }

    /// The ceiling at `y = height`, filleted into the walls by `CORNER_RADIUS`.
    fn ceiling_distance(&self, p: Vec3) -> f64 {
        let r = CORNER_RADIUS;
        let qx = (p.x.abs() - (self.half_x - r)).max(0.0);
        let qz = (p.z.abs() - (self.half_z - r)).max(0.0);
        let qy = (p.y - (self.height - r)).max(0.0);
        (qx * qx + qy * qy + qz * qz).sqrt() - r
    }

    /// Gradient of `wall_distance`, which points away from the interior.
    fn wall_gradient(&self, p: Vec3) -> Vec3 {
        let r = CORNER_RADIUS;
        let qx = p.x.abs() - (self.half_x - r);
        let qz = p.z.abs() - (self.half_z - r);
        let sx = if p.x >= 0.0 { 1.0 } else { -1.0 };
        let sz = if p.z >= 0.0 { 1.0 } else { -1.0 };
        if qx > 0.0 || qz > 0.0 {
            // Outside the shrunk rectangle: a corner/edge distance.
            let ox = qx.max(0.0);
            let oz = qz.max(0.0);
            let l = (ox * ox + oz * oz).sqrt();
            Vec3::new(ox * sx / l, 0.0, oz * sz / l)
        } else if qx >= qz {
            // Inside: the nearest face is the x wall.
            Vec3::new(sx, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 0.0, sz)
        }
    }

    /// Gradient of `floor_distance`.
    fn floor_gradient(&self, p: Vec3) -> Vec3 {
        let r = FLOOR_FILLET;
        let qx = (p.x.abs() - (self.half_x - r)).max(0.0);
        let qz = (p.z.abs() - (self.half_z - r)).max(0.0);
        let qy = (r - p.y).max(0.0);
        let sx = if p.x >= 0.0 { 1.0 } else { -1.0 };
        let sz = if p.z >= 0.0 { 1.0 } else { -1.0 };
        let l = (qx * qx + qy * qy + qz * qz).sqrt();
        if l > 1e-12 {
            Vec3::new(qx * sx / l, -qy / l, qz * sz / l)
        } else {
            Vec3::new(0.0, -1.0, 0.0)
        }
    }

    /// Gradient of `ceiling_distance`.
    fn ceiling_gradient(&self, p: Vec3) -> Vec3 {
        let r = CORNER_RADIUS;
        let qx = (p.x.abs() - (self.half_x - r)).max(0.0);
        let qz = (p.z.abs() - (self.half_z - r)).max(0.0);
        let qy = (p.y - (self.height - r)).max(0.0);
        let sx = if p.x >= 0.0 { 1.0 } else { -1.0 };
        let sz = if p.z >= 0.0 { 1.0 } else { -1.0 };
        let l = (qx * qx + qy * qy + qz * qz).sqrt();
        if l > 1e-12 {
            Vec3::new(qx * sx / l, qy / l, qz * sz / l)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        }
    }

    pub fn contains(&self, p: Vec3, margin: f64) -> bool {
        p.x.abs() <= self.half_x + margin
            && p.z.abs() <= self.half_z + margin
            && p.y >= -margin
            && p.y <= self.height + margin
    }

    /// The goal a ball of radius `radius` centred at `p` has scored in, if its
    /// centre is past the goal line and within the mouth. Following RocketSim,
    /// the line is `GOAL_SCORE_BASE_Z + radius` (the ball must have crossed
    /// fully). `Goal::Minus` is the `-Z` (blue) goal.
    pub fn ball_in_goal(&self, p: Vec3, radius: f64) -> Option<Goal> {
        if p.z.abs() > GOAL_SCORE_BASE_Z + radius
            && p.x.abs() <= GOAL_HALF_WIDTH
            && p.y <= GOAL_HEIGHT
        {
            Some(if p.z < 0.0 { Goal::Minus } else { Goal::Plus })
        } else {
            None
        }
    }
}

/// The `(value, gradient)` pair with the largest value (the nearer surface of two
/// convex constraints), for the `max` intersection of signed distance fields.
fn max2(a: (f64, Vec3), b: (f64, Vec3)) -> (f64, Vec3) {
    if b.0 > a.0 {
        b
    } else {
        a
    }
}

/// The `(value, gradient)` pair with the largest value: the nearest surface among
/// convex constraints (the standard `max` intersection of signed distance
/// fields), together with its outward gradient as the other pair member.
fn max3(a: (f64, Vec3), b: (f64, Vec3), c: (f64, Vec3)) -> (f64, Vec3) {
    let mut best = a;
    if b.0 > best.0 {
        best = b;
    }
    if c.0 > best.0 {
        best = c;
    }
    best
}

/// `min` of two fields (their union), with the concave corner where they meet
/// rounded off over a radius `k`. The sharp corner a plain `min` leaves is what a
/// point-contact car jams on. Polynomial, so it is deterministic and libm-free.
fn smin(a: (f64, Vec3), b: (f64, Vec3), k: f64) -> (f64, Vec3) {
    let h = (0.5 + 0.5 * (b.0 - a.0) / k).clamp(0.0, 1.0);
    let value = b.0 + (a.0 - b.0) * h - k * h * (1.0 - h);
    let grad = b.1 + (a.1 - b.1) * h;
    (value, grad)
}

/// `max` of two fields (their intersection), with the convex corner filled in
/// over a radius `k`. The [`smin`] counterpart - here the fillet replaces the
/// sharp corner a plain `max` leaves. Polynomial, so it is deterministic.
fn smax(a: (f64, Vec3), b: (f64, Vec3), k: f64) -> (f64, Vec3) {
    let h = (0.5 + 0.5 * (a.0 - b.0) / k).clamp(0.0, 1.0);
    let value = b.0 + (a.0 - b.0) * h + k * h * (1.0 - h);
    let grad = b.1 + (a.1 - b.1) * h;
    (value, grad)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floor_and_ceiling_are_flat_and_at_the_right_height() {
        let a = Arena::default();
        // On the floor and just above it, the field is exactly the height (the
        // distance saturates further out, but nothing ever contacts from there).
        assert!((a.inside_distance(Vec3::new(0.0, 0.0, 0.0))).abs() < 1e-12);
        assert!((a.inside_distance(Vec3::new(0.0, 0.3, 0.0)) + 0.3).abs() < 1e-12);
        // The ceiling.
        let top = Vec3::new(0.0, a.height, 0.0);
        assert!(a.inside_distance(top).abs() < 1e-12);
        assert!((a.inside_distance(Vec3::new(0.0, a.height - 0.4, 0.0)) + 0.4).abs() < 1e-12);
        assert!((a.inside_distance(Vec3::new(0.0, a.height + 3.0, 0.0)) - 3.0).abs() < 1e-12);
        // Inside is negative, outside positive.
        assert!(a.inside_distance(Vec3::new(0.0, 10.0, 0.0)) < 0.0);
        assert!(a.inside_distance(Vec3::new(0.0, -1.0, 0.0)) > 0.0);
    }

    #[test]
    fn normals_point_into_the_arena() {
        let a = Arena::default();
        // Floor: up.
        let n = a.normal_at(Vec3::new(0.0, 0.0, 0.0));
        assert!((n - Vec3::UP).length() < 1e-9, "{n:?}");
        // Ceiling: down.
        let n = a.normal_at(Vec3::new(0.0, a.height, 0.0));
        assert!((n + Vec3::UP).length() < 1e-9, "{n:?}");
        // +X wall, in the middle of the face: -X.
        let n = a.normal_at(Vec3::new(a.half_x, 10.0, 0.0));
        assert!((n - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-9, "{n:?}");
        // -Z wall: +Z.
        let n = a.normal_at(Vec3::new(0.0, 10.0, -a.half_z));
        assert!((n - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9, "{n:?}");
    }

    #[test]
    fn vertical_corners_are_chamfered() {
        let a = Arena::default();
        // The sharp box corner is outside the rounded arena, so the car cannot
        // wedge itself into it.
        let sharp = Vec3::new(a.half_x, 10.0, a.half_z);
        assert!(
            a.inside_distance(sharp) > 0.5,
            "corner should be cut off, got {}",
            a.inside_distance(sharp)
        );
        // And the normal there is diagonal, halfway between the two walls.
        let n = a.normal_at(sharp);
        assert!(n.x < -0.3 && n.z < -0.3, "corner normal {n:?}");
    }

    #[test]
    fn wall_to_ceiling_join_is_smooth() {
        // Walk up a wall and over the rim; the normal must rotate from
        // horizontal to straight down without ever flipping back.
        let a = Arena::default();
        let mut last = a.normal_at(Vec3::new(a.half_x - 0.001, 1.0, 0.0));
        for i in 1..=200 {
            let y = 1.0 + (a.height - 1.5) * (i as f64) / 200.0;
            let n = a.normal_at(Vec3::new(a.half_x - 0.001, y, 0.0));
            // Up the wall the normal tilts steadily from horizontal to down, so
            // its y component must fall monotonically.
            assert!(n.y <= last.y + 1e-9, "normal turned back at y={y}");
            last = n;
        }
        // By the ceiling the normal points straight down.
        let top = a.normal_at(Vec3::new(a.half_x - 0.001, a.height - 0.001, 0.0));
        assert!(top.y < -0.5, "ceiling normal {top:?}");
    }
}
