//! The ball: a sphere with linear velocity and spin.
//!
//! The model is real rigid-body physics for a single sphere, just written out
//! by hand: gravity, Rocket League's linear drag, the Magnus force from spin,
//! and impulse-based contact resolution against the arena surface with Coulomb
//! friction that is allowed to convert sliding into spin. That friction term
//! is what gives you curve off the wall and "pinch" behaviour, and because we
//! control it directly it is reproducible rather than emergent.

use crate::arena::Arena;
use crate::math::{StateHasher, Vec3};

// All values below are lifted from RocketSim's `RLConst.h`, which reverse-
// engineered them from the shipped game. Unreal units are 1 cm, so the
// conversions are exact divisions by 100.

/// `BALL_COLLISION_RADIUS_SOCCAR = 91.25 uu`. This is the *collision* radius;
/// the ball you see is slightly larger and Rocket League rests it at 93.15 uu
/// to account for the collision margin.
pub const BALL_RADIUS: f64 = 0.9125;
/// `BALL_MASS_BT = CAR_MASS_BT / 6`: Rocket League's ball is one sixth of a car.
pub const BALL_MASS: f64 = 30.0;
/// Ball linear damping. RocketSim sets the Bullet body's `linearDamping` to
/// `BALL_DRAG = 0.03`, so the ball keeps `0.97` of its speed per second. The
/// per-tick factor is `(1 - 0.03)^(1/120)`, precomputed so the step needs no
/// `pow` (which is a platform libm call and would break determinism).
pub const BALL_DRAG_KEEP: f64 = 0.999_746_205_482_239_6;
/// Magnus coefficient: a = k * (spin x velocity). RocketSim/Bullet do *not*
/// model this — a freely-spinning sphere gets no aerodynamic force there — but
/// Rocket League balls do curve, so it is kept as a deliberate extra.
pub const BALL_MAGNUS: f64 = 0.010;
/// `BALL_RESTITUTION = 0.6`.
pub const BALL_WALL_RESTITUTION: f64 = 0.6;
/// `BALL_FRICTION = 0.35`.
pub const BALL_WALL_FRICTION: f64 = 0.35;
/// `BALL_MAX_SPEED = 6000 uu/s`.
pub const BALL_MAX_SPEED: f64 = 60.0;
/// `BALL_MAX_ANG_SPEED = 6 rad/s`. The ball can never spin faster than this.
pub const BALL_MAX_ANG_SPEED: f64 = 6.0;

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

    /// Acceleration from gravity + Magnus. Drag is *not* here: Rocket League
    /// models it as Bullet linear damping, which scales the velocity rather
    /// than adding an acceleration, so it is applied in `step`.
    pub fn acceleration(&self, gravity: f64) -> Vec3 {
        let magnus = self.spin.cross(self.vel) * BALL_MAGNUS;
        Vec3::new(0.0, gravity, 0.0) + magnus
    }

    pub fn step(&mut self, dt: f64, gravity: f64, arena: &Arena) {
        self.vel += self.acceleration(gravity) * dt;
        // Linear damping, applied the way Bullet does: a per-tick velocity
        // multiplier of `(1 - damping)^dt`.
        self.vel = self.vel * BALL_DRAG_KEEP;
        self.pos += self.vel * dt;
        self.collide_arena(arena);
        self.clamp_speeds();
    }

    /// Apply Rocket League's hard velocity caps. `BALL_MAX_SPEED` is a real
    /// thing in the game (RocketSim clamps the rigid body to it every tick) and
    /// is what bounds the absurd ball speeds a pinch can otherwise produce.
    pub fn clamp_speeds(&mut self) {
        let speed = self.vel.length();
        if speed > BALL_MAX_SPEED {
            self.vel = self.vel * (BALL_MAX_SPEED / speed);
        }
        let spin = self.spin.length();
        if spin > BALL_MAX_ANG_SPEED {
            self.spin = self.spin * (BALL_MAX_ANG_SPEED / spin);
        }
    }

    pub fn collide_arena(&mut self, arena: &Arena) {
        // The ball is a sphere, so a contact is just "the centre is closer to
        // the surface than the radius", against whatever piece of the rounded
        // enclosure is nearest.
        let distance = arena.inside_distance(self.pos);
        let penetration = self.radius + distance;
        if penetration <= 0.0 {
            return;
        }
        let normal = arena.normal_at(self.pos);
        self.resolve_surface(normal, penetration);
    }

    /// Resolve one surface contact, touching both linear and angular state.
    /// `normal` points into the playable space, so the ball is pushed out along
    /// it.
    fn resolve_surface(&mut self, normal: Vec3, penetration: f64) {
        // Push out of the surface.
        self.pos += normal * penetration;

        // Vector from the ball's centre to the contact point.
        let r = normal * -self.radius;
        // Velocity of the material point of the ball at the contact.
        let contact_vel = self.vel + self.spin.cross(r);
        let vn = contact_vel.dot(normal);
        if vn >= 0.0 {
            return; // Already separating.
        }

        // Normal impulse. The surface is immovable, so only the ball's mass
        // enters the reduced mass.
        let jn = -(1.0 + BALL_WALL_RESTITUTION) * vn * self.mass;
        self.vel += normal * (jn / self.mass);

        // Tangential (friction) impulse, clamped by Coulomb's law.
        let tangent_vel = contact_vel - normal * vn;
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
