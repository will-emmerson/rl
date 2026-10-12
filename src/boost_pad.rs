//! Boost pads: the six big (100 boost) and 28 small (12 boost) pickups on the
//! Soccar pitch.
//!
//! Positions and constants come from RocketSim's `RLConst.h` (`BoostPads`), in
//! Unreal units (1 uu = 1 cm) converted to metres. Rocket League's pads are
//! cylinders (big) and boxes (small) with bespoke trigger volumes; here a pad is
//! just a horizontal pickup radius, which is all the sim needs.

use crate::math::{StateHasher, Vec3};

/// `CYL_RAD_BIG = 208 uu`: big-pad pickup radius.
pub const PAD_RADIUS_BIG: f64 = 2.08;
/// `CYL_RAD_SMALL = 144 uu`: small-pad pickup radius.
pub const PAD_RADIUS_SMALL: f64 = 1.44;
/// `BOOST_AMOUNT_BIG`.
pub const PAD_AMOUNT_BIG: f64 = 100.0;
/// `BOOST_AMOUNT_SMALL`.
pub const PAD_AMOUNT_SMALL: f64 = 12.0;
/// `COOLDOWN_BIG`, in seconds.
pub const PAD_COOLDOWN_BIG: f64 = 10.0;
/// `COOLDOWN_SMALL`, in seconds.
pub const PAD_COOLDOWN_SMALL: f64 = 4.0;

/// The car must be this close to the floor to take a pad: driving over it
/// counts, flying well above it does not. (RL tests the car's hitbox against the
/// pad's trigger volume; this is the cheap stand-in.)
const PAD_PICKUP_MAX_HEIGHT: f64 = 1.0;

#[derive(Clone, Copy, Debug)]
pub struct BoostPad {
    pub pos: Vec3,
    pub amount: f64,
    pub cooldown_duration: f64,
    /// Horizontal pickup radius.
    pub radius: f64,
    pub is_big: bool,
    /// Seconds until it is available again; `<= 0` means available.
    pub cooldown: f64,
}

impl BoostPad {
    pub fn available(&self) -> bool {
        self.cooldown <= 0.0
    }

    /// Advance the cooldown by one tick.
    pub fn tick(&mut self, dt: f64) {
        if self.cooldown > 0.0 {
            self.cooldown = (self.cooldown - dt).max(0.0);
        }
    }

    /// True if a car centred at `car_pos` is over this pad (and low enough).
    pub fn covers(&self, car_pos: Vec3) -> bool {
        if car_pos.y > PAD_PICKUP_MAX_HEIGHT {
            return false;
        }
        let dx = car_pos.x - self.pos.x;
        let dz = car_pos.z - self.pos.z;
        dx * dx + dz * dz <= self.radius * self.radius
    }

    /// Only the cooldown changes, so that is all the hash needs.
    pub fn hash_into(&self, h: &mut StateHasher) {
        h.fold(self.cooldown);
    }
}

/// RocketSim `BoostPads::LOCS_SMALL_SOCCAR`, `(x, y, z)` in uu.
const SMALL_LOCS_UU: [(f64, f64, f64); 28] = [
    (0.0, -4240.0, 70.0),
    (-1792.0, -4184.0, 70.0),
    (1792.0, -4184.0, 70.0),
    (-940.0, -3308.0, 70.0),
    (940.0, -3308.0, 70.0),
    (0.0, -2816.0, 70.0),
    (-3584.0, -2484.0, 70.0),
    (3584.0, -2484.0, 70.0),
    (-1788.0, -2300.0, 70.0),
    (1788.0, -2300.0, 70.0),
    (-2048.0, -1036.0, 70.0),
    (0.0, -1024.0, 70.0),
    (2048.0, -1036.0, 70.0),
    (-1024.0, 0.0, 70.0),
    (1024.0, 0.0, 70.0),
    (-2048.0, 1036.0, 70.0),
    (0.0, 1024.0, 70.0),
    (2048.0, 1036.0, 70.0),
    (-1788.0, 2300.0, 70.0),
    (1788.0, 2300.0, 70.0),
    (-3584.0, 2484.0, 70.0),
    (3584.0, 2484.0, 70.0),
    (0.0, 2816.0, 70.0),
    (-940.0, 3308.0, 70.0),
    (940.0, 3308.0, 70.0),
    (-1792.0, 4184.0, 70.0),
    (1792.0, 4184.0, 70.0),
    (0.0, 4240.0, 70.0),
];

/// RocketSim `BoostPads::LOCS_BIG_SOCCAR`, `(x, y, z)` in uu.
const BIG_LOCS_UU: [(f64, f64, f64); 6] = [
    (-3584.0, 0.0, 73.0),
    (3584.0, 0.0, 73.0),
    (-3072.0, 4096.0, 73.0),
    (3072.0, 4096.0, 73.0),
    (-3072.0, -4096.0, 73.0),
    (3072.0, -4096.0, 73.0),
];

/// RL coordinates are `(x = width, y = length, z = up)`; the sim uses
/// `(x = width, y = up, z = length)`. Both in metres here.
fn to_sim(x_uu: f64, y_uu: f64, z_uu: f64) -> Vec3 {
    Vec3::new(x_uu / 100.0, z_uu / 100.0, y_uu / 100.0)
}

/// The standard Soccar pad layout: the 28 small pads then the 6 big ones, in a
/// fixed order so iteration is deterministic.
pub fn soccar() -> Vec<BoostPad> {
    let mut pads = Vec::with_capacity(SMALL_LOCS_UU.len() + BIG_LOCS_UU.len());
    for &(x, y, z) in &SMALL_LOCS_UU {
        pads.push(BoostPad {
            pos: to_sim(x, y, z),
            amount: PAD_AMOUNT_SMALL,
            cooldown_duration: PAD_COOLDOWN_SMALL,
            radius: PAD_RADIUS_SMALL,
            is_big: false,
            cooldown: 0.0,
        });
    }
    for &(x, y, z) in &BIG_LOCS_UU {
        pads.push(BoostPad {
            pos: to_sim(x, y, z),
            amount: PAD_AMOUNT_BIG,
            cooldown_duration: PAD_COOLDOWN_BIG,
            radius: PAD_RADIUS_BIG,
            is_big: true,
            cooldown: 0.0,
        });
    }
    pads
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soccar_layout_has_the_right_pads() {
        let pads = soccar();
        assert_eq!(pads.len(), 34);
        assert_eq!(pads.iter().filter(|p| p.is_big).count(), 6);
        assert_eq!(pads.iter().filter(|p| !p.is_big).count(), 28);
        // A big corner pad, in metres: RL (-3072, -4096, 73) uu.
        assert!(pads
            .iter()
            .any(|p| p.is_big && (p.pos.x + 30.72).abs() < 1e-9 && (p.pos.z + 40.96).abs() < 1e-9));
    }
}
