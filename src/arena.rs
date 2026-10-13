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

use crate::math::Vec3;

/// Radius of the vertical corner chamfer and the wall-to-ceiling fillet, in
/// metres.
pub const CORNER_RADIUS: f64 = 4.0;
/// Radius of the wall-to-floor fillet, in metres. Deliberately much smaller
/// than `CORNER_RADIUS`: a big fillet here would eat into the playable floor
/// near every wall.
pub const FLOOR_FILLET: f64 = 1.0;

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
    /// Signed distance to the arena surface: negative inside the playable
    /// space, positive outside, zero exactly on a surface.
    pub fn inside_distance(&self, p: Vec3) -> f64 {
        self.wall_distance(p)
            .max(self.floor_distance(p))
            .max(self.ceiling_distance(p))
    }

    /// Unit normal pointing *into* the playable space at `p`, i.e. the direction
    /// an object resting on the surface is pushed. Over the rounded corners the
    /// field is smooth, so there is no face to choose between.
    pub fn normal_at(&self, p: Vec3) -> Vec3 {
        let w = self.wall_distance(p);
        let f = self.floor_distance(p);
        let c = self.ceiling_distance(p);
        // The active constraint is the one whose field is greatest (closest to
        // the surface coming from inside); its negated gradient is the inward
        // normal.
        let gradient = if w >= f && w >= c {
            self.wall_gradient(p)
        } else if f >= c {
            self.floor_gradient(p)
        } else {
            self.ceiling_gradient(p)
        };
        let normal = -gradient;
        if normal.length_squared() > 1e-18 {
            normal.normalized()
        } else {
            Vec3::UP
        }
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
