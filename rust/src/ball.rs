//! The ball: a sphere with linear velocity and spin.
//!
//! The model is real rigid-body physics for a single sphere, just written out
//! by hand: gravity, quadratic drag, the Magnus force from spin, and
//! impulse-based contact resolution against the arena planes with Coulomb
//! friction that is allowed to convert sliding into spin. That friction term
//! is what gives you curve off the wall and "pinch" behaviour, and because we
//! control it directly it is reproducible rather than emergent.

use crate::arena::Arena;
use crate::math::{StateHasher, Vec3};

/// Community-derived Rocket League scale, in metres.
pub const BALL_RADIUS: f64 = 0.93;
pub const BALL_MASS: f64 = 1.5;
/// Quadratic drag coefficient: a = -k * |v| * v.
pub const BALL_DRAG: f64 = 0.0009;
/// Magnus coefficient: a = k * (spin x velocity).
pub const BALL_MAGNUS: f64 = 0.010;
/// Spin retained per 120 Hz tick.
pub const BALL_SPIN_KEEP: f64 = 0.999;
pub const BALL_WALL_RESTITUTION: f64 = 0.62;
pub const BALL_WALL_FRICTION: f64 = 0.45;

#[derive(Clone, Copy, Debug)]
pub struct Ball {
    pub pos: Vec3,
    pub vel: Vec3,
    /// Angular velocity in rad/s.
    pub spin: Vec3,
    pub radius: f64,
    pub mass: f64,
}

impl Default for Ball {
    fn default() -> Self {
        Self {
            pos: Vec3::new(0.0, BALL_RADIUS, 0.0),
            vel: Vec3::ZERO,
            spin: Vec3::ZERO,
            radius: BALL_RADIUS,
            mass: BALL_MASS,
        }
    }
}

impl Ball {
    /// Moment of inertia of a solid sphere.
    pub fn inertia(&self) -> f64 {
        0.4 * self.mass * self.radius * self.radius
    }

    /// Acceleration from gravity + drag + Magnus.
    pub fn acceleration(&self, gravity: f64) -> Vec3 {
        let speed = self.vel.length();
        let drag = self.vel * (-BALL_DRAG * speed);
        let magnus = self.spin.cross(self.vel) * BALL_MAGNUS;
        Vec3::new(0.0, gravity, 0.0) + drag + magnus
    }

    pub fn step(&mut self, dt: f64, gravity: f64, arena: &Arena) {
        self.vel += self.acceleration(gravity) * dt;
        self.pos += self.vel * dt;
        self.spin = self.spin * BALL_SPIN_KEEP;
        self.collide_arena(arena);
    }

    pub fn collide_arena(&mut self, arena: &Arena) {
        for plane in arena.planes() {
            self.resolve_plane(&plane);
        }
    }

    /// Resolve one plane contact, touching both linear and angular state.
    fn resolve_plane(&mut self, plane: &crate::arena::Plane) {
        let distance = plane.distance(self.pos);
        let penetration = self.radius - distance;
        if penetration <= 0.0 {
            return;
        }

        // Push out of the plane.
        self.pos += plane.normal * penetration;

        // Vector from the ball's centre to the contact point.
        let r = plane.normal * -self.radius;
        // Velocity of the material point of the ball at the contact.
        let contact_vel = self.vel + self.spin.cross(r);
        let vn = contact_vel.dot(plane.normal);
        if vn >= 0.0 {
            return; // Already separating.
        }

        // Normal impulse. The plane is immovable, so only the ball's mass
        // enters the reduced mass.
        let jn = -(1.0 + BALL_WALL_RESTITUTION) * vn * self.mass;
        self.vel += plane.normal * (jn / self.mass);

        // Tangential (friction) impulse, clamped by Coulomb's law.
        let tangent_vel = contact_vel - plane.normal * vn;
        let tangent_speed = tangent_vel.length();
        if tangent_speed <= 1e-9 {
            return;
        }
        let tangent = tangent_vel / tangent_speed;

        // Impulse that would exactly stop the contact point sliding. For a
        // solid sphere that is (2/7) * m * v_t.
        let stop_slide = (2.0 / 7.0) * self.mass * tangent_speed;
        let max_friction = BALL_WALL_FRICTION * jn.abs();
        let magnitude = stop_slide.min(max_friction);
        let impulse = tangent * -magnitude;

        self.vel += impulse / self.mass;
        self.spin += r.cross(impulse) / self.inertia();
    }

    /// Resolve a collision against an oriented box, used for car contacts.
    ///
    /// `box_pos`/`box_axes` describe the car; `box_half` its half extents.
    /// `box_vel` is the car's linear velocity and `box_spin` its angular
    /// velocity, both needed so a moving car can "flick" the ball.
    /// Returns the world-space contact normal and the impulse magnitude if a
    /// contact happened.
    #[allow(clippy::too_many_arguments)]
    pub fn resolve_obb(
        &mut self,
        box_pos: Vec3,
        axes: [Vec3; 3],
        box_half: Vec3,
        box_vel: Vec3,
        box_spin: Vec3,
        box_mass: f64,
        restitution: f64,
        friction: f64,
    ) -> Option<(Vec3, f64)> {
        let delta = self.pos - box_pos;
        let local = Vec3::new(
            delta.dot(axes[0]).clamp(-box_half.x, box_half.x),
            delta.dot(axes[1]).clamp(-box_half.y, box_half.y),
            delta.dot(axes[2]).clamp(-box_half.z, box_half.z),
        );
        let closest = box_pos + axes[0] * local.x + axes[1] * local.y + axes[2] * local.z;

        let offset = self.pos - closest;
        let distance = offset.length();
        let penetration = self.radius - distance;
        if penetration <= 0.0 {
            return None;
        }

        // Degenerate case: the centre is inside the box. Push out along the
        // least-penetrated axis.
        let normal = if distance > 1e-9 {
            offset / distance
        } else {
            let gaps = Vec3::new(
                box_half.x - (delta.dot(axes[0])).abs(),
                box_half.y - (delta.dot(axes[1])).abs(),
                box_half.z - (delta.dot(axes[2])).abs(),
            );
            let mut axis = 0;
            if gaps.y < gaps.x {
                axis = 1;
            }
            if gaps.z < gaps.get(axis) {
                axis = 2;
            }
            let sign = if delta.dot(axes[axis]) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            axes[axis] * sign
        };

        self.pos += normal * penetration;

        let r_ball = normal * -self.radius;
        let ball_contact_vel = self.vel + self.spin.cross(r_ball);
        let car_contact_vel = box_vel + box_spin.cross(closest - box_pos);
        let relative = ball_contact_vel - car_contact_vel;
        let vn = relative.dot(normal);
        if vn >= 0.0 {
            return Some((normal, 0.0));
        }

        let inv_ball = 1.0 / self.mass;
        let inv_box = if box_mass.is_finite() && box_mass > 0.0 {
            1.0 / box_mass
        } else {
            0.0
        };
        let jn = -(1.0 + restitution) * vn / (inv_ball + inv_box);
        self.vel += normal * (jn * inv_ball);

        // Friction on the ball's surface, which turns a corner hit into spin.
        let tangent_vel = relative - normal * vn;
        let tangent_speed = tangent_vel.length();
        if tangent_speed > 1e-9 {
            let tangent = tangent_vel / tangent_speed;
            let stop_slide = (2.0 / 7.0) * self.mass * tangent_speed;
            let max_friction = friction * jn.abs();
            let impulse = tangent * -stop_slide.min(max_friction);
            self.vel += impulse / self.mass;
            self.spin += r_ball.cross(impulse) / self.inertia();
        }

        Some((normal, jn))
    }

    pub fn hash_into(&self, h: &mut StateHasher) {
        self.pos.hash_into(h);
        self.vel.hash_into(h);
        self.spin.hash_into(h);
    }
}

// Small helper so `Vec3` indexing reads cleanly in the OBB routine.
trait Vec3Index {
    fn get(&self, axis: usize) -> f64;
}

impl Vec3Index for Vec3 {
    fn get(&self, axis: usize) -> f64 {
        match axis {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}
