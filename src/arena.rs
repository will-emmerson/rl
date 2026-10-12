//! The arena: an axis-aligned box. A fixed set of static planes is all the
//! geometry the simulation ever needs, which is exactly why a general
//! broadphase/contact solver is overkill here.
//!
//! Dimensions come from RocketSim's `RLConst.h` (`ARENA_EXTENT_X/Y`,
//! `ARENA_HEIGHT`), which are in Unreal units (1 uu = 1 cm). Real Rocket League
//! uses a collision *mesh* (dumped with RLArenaCollisionDumper): the walls are
//! chamfered near the corners and the ceiling is domed, and the goals cut into
//! the end walls. A plain box is a deliberate approximation of that — it gets
//! the playable extents right, which is what the physics here needs.

use crate::math::Vec3;

/// A static collision plane. `normal` points *into* playable space, so a
/// sphere's centre must stay at a signed distance of at least `radius`.
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
}

impl Plane {
    /// Signed distance from the plane along its normal.
    pub fn distance(&self, p: Vec3) -> f64 {
        (p - self.point).dot(self.normal)
    }
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
    /// Floor then ceiling then the four walls. The order is fixed and the
    /// resolution is sequential, so it is fully reproducible.
    pub fn planes(&self) -> [Plane; 6] {
        [
            Plane {
                point: Vec3::new(0.0, 0.0, 0.0),
                normal: Vec3::new(0.0, 1.0, 0.0),
            },
            Plane {
                point: Vec3::new(0.0, self.height, 0.0),
                normal: Vec3::new(0.0, -1.0, 0.0),
            },
            Plane {
                point: Vec3::new(-self.half_x, 0.0, 0.0),
                normal: Vec3::new(1.0, 0.0, 0.0),
            },
            Plane {
                point: Vec3::new(self.half_x, 0.0, 0.0),
                normal: Vec3::new(-1.0, 0.0, 0.0),
            },
            Plane {
                point: Vec3::new(0.0, 0.0, -self.half_z),
                normal: Vec3::new(0.0, 0.0, 1.0),
            },
            Plane {
                point: Vec3::new(0.0, 0.0, self.half_z),
                normal: Vec3::new(0.0, 0.0, -1.0),
            },
        ]
    }

    /// The four vertical walls, used to keep the car inside.
    pub fn wall_planes(&self) -> [Plane; 4] {
        let p = self.planes();
        [p[2], p[3], p[4], p[5]]
    }

    pub fn contains(&self, p: Vec3, margin: f64) -> bool {
        p.x.abs() <= self.half_x + margin
            && p.z.abs() <= self.half_z + margin
            && p.y >= -margin
            && p.y <= self.height + margin
    }
}
