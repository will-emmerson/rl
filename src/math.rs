//! Minimal vector maths.
//!
//! Everything is `f64` and every operation is plain arithmetic plus `sqrt`.
//! `sqrt` is correctly rounded by IEEE-754, so it is bit-identical on every
//! conforming platform. There are deliberately no trigonometric functions
//! anywhere in the simulation step: the car's heading is carried as a unit
//! vector and rotated incrementally instead. That matters for determinism —
//! see `docs/PHYSICS.md`.

use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    pub const UP: Self = Self::new(0.0, 1.0, 0.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Drop the Y component. Used to keep the car's maths in the ground plane.
    pub fn xz(self) -> Self {
        Self::new(self.x, 0.0, self.z)
    }

    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    pub fn distance(self, o: Self) -> f64 {
        (self - o).length()
    }

    /// Unit vector, or zero if the input is (near) zero.
    pub fn normalized(self) -> Self {
        let len = self.length();
        if len > 1e-12 {
            self / len
        } else {
            Self::ZERO
        }
    }

    pub fn min(self, o: Self) -> Self {
        Self::new(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }

    pub fn max(self, o: Self) -> Self {
        Self::new(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }

    pub fn clamp(self, lo: Self, hi: Self) -> Self {
        self.max(lo).min(hi)
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// Feed the raw bit pattern into a hash. `-0.0` is canonicalised to `0.0`
    /// so that a sign flip on zero cannot show up as a false divergence.
    pub fn hash_into(self, h: &mut StateHasher) {
        h.fold(self.x);
        h.fold(self.y);
        h.fold(self.z);
    }
}

/// Deterministic `(sin, cos)` for the small angles the simulation uses.
///
/// Rust's `f64::sin`/`cos` delegate to the platform libm, and libm results are
/// *not* guaranteed identical across platforms — which would silently break
/// the determinism guarantee. These Taylor polynomials are pure arithmetic and
/// agree with the true value to a few parts in 1e-8 across the range used
/// here (|x| <= 0.5).
fn sin_cos_small(x: f64) -> (f64, f64) {
    let x2 = x * x;
    let x4 = x2 * x2;
    let x6 = x4 * x2;
    // sin x = x - x^3/6 + x^5/120 - x^7/5040
    let sin = x * (1.0 - x2 / 6.0 + x4 / 120.0 - x6 / 5040.0);
    // cos x = 1 - x^2/2 + x^4/24 - x^6/720
    let cos = 1.0 - x2 / 2.0 + x4 / 24.0 - x6 / 720.0;
    (sin, cos)
}

/// Rotate a horizontal unit vector `f` by `angle` radians about +Y, the
/// direction that turns a car towards its left.
///
/// Uses only `+ - * / sqrt`, so it is bit-reproducible across platforms. Large
/// angles are split into sub-steps to keep the polynomial inside its accurate
/// range; the sub-step count is a deterministic function of the input.
pub fn rotate_y(f: Vec3, angle: f64) -> Vec3 {
    let steps = ((angle.abs() / 0.5).ceil() as i64).max(1);
    let step = angle / steps as f64;
    let (sin, cos) = sin_cos_small(step);
    let mut result = f;
    for _ in 0..steps {
        let perp = Vec3::new(result.z, 0.0, -result.x);
        result = (result * cos + perp * sin).normalized();
    }
    result
}

/// Forward vector for a heading, exposed for tests and views.
pub fn heading_forward(heading: Vec3) -> Vec3 {
    heading.normalized()
}

/// Right vector for a heading (forward x up).
pub fn heading_right(heading: Vec3) -> Vec3 {
    heading.normalized().cross(Vec3::UP)
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, o: Self) {
        *self = *self + o;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, o: Self) {
        *self = *self - o;
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// Incremental 64-bit hash used for divergence detection.
///
/// Not cryptographic — it just needs a strong avalanche so that one differing
/// float bit is overwhelmingly likely to change the final value.
#[derive(Clone, Copy, Debug)]
pub struct StateHasher(pub u64);

impl Default for StateHasher {
    fn default() -> Self {
        Self::new()
    }
}

impl StateHasher {
    pub fn new() -> Self {
        Self(0x9E37_79B9_7F4A_7C15)
    }

    pub fn fold(&mut self, v: f64) {
        let bits = if v == 0.0 { 0 } else { v.to_bits() };
        self.0 = mix64(self.0 ^ bits);
    }

    pub fn fold_u64(&mut self, v: u64) {
        self.0 = mix64(self.0 ^ v);
    }
}

fn mix64(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotate_y_turns_left_for_positive_angle() {
        // Heading +Z (yaw 0). Turning left must move the nose towards +X.
        let f = Vec3::new(0.0, 0.0, 1.0);
        let g = rotate_y(f, 0.1);
        assert!(g.x > 0.0, "expected +X component, got {g:?}");
        assert!((g.length() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn rotate_y_matches_axis_convention_exactly() {
        let f = Vec3::new(0.0, 0.0, 1.0);
        let g = rotate_y(f, std::f64::consts::FRAC_PI_2);
        assert!((g.x - 1.0).abs() < 1e-6, "got {g:?}");
        assert!(g.z.abs() < 1e-6, "got {g:?}");
    }

    #[test]
    fn rotate_y_is_accurate_at_step_sized_angles() {
        // The sim only ever asks for yaw_rate * DT <= 0.05 rad.
        let g = rotate_y(Vec3::new(0.0, 0.0, 1.0), 0.05);
        assert!((g.x - 0.049_979_169_270_678_34).abs() < 1e-12, "got {g:?}");
        assert!((g.z - 0.998_750_260_394_966_3).abs() < 1e-12, "got {g:?}");
    }

    #[test]
    fn full_turn_returns_to_start() {
        let start = Vec3::new(0.0, 0.0, 1.0);
        let mut f = start;
        for _ in 0..1200 {
            f = rotate_y(f, std::f64::consts::TAU / 1200.0);
        }
        assert!((f - start).length() < 1e-9, "drifted to {f:?}");
    }

    #[test]
    fn heading_right_is_perpendicular() {
        let f = Vec3::new(0.0, 0.0, 1.0);
        let r = heading_right(f);
        assert!(r.dot(f).abs() < 1e-12);
        // Facing +Z with +Y up, "right" is -X.
        assert!(r.x < 0.0, "got {r:?}");
    }

    #[test]
    fn hash_canonicalises_negative_zero() {
        let mut a = StateHasher::new();
        a.fold(0.0);
        let mut b = StateHasher::new();
        b.fold(-0.0);
        assert_eq!(a.0, b.0);
    }
}
