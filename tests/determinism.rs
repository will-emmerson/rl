//! Determinism: the whole reason for hand-rolling this.
//!
//! These tests are the executable form of the claim that a given input stream
//! produces a bit-identical state stream.

use rl_sim::{Input, World, TICK_HZ};

/// A fixed, non-trivial input stream. Pure function of the tick.
fn scripted(tick: u32) -> Input {
    let phase = tick % 480;
    match phase {
        0..=180 => Input::new(1.0, 0.0, false, false, false),
        181..=240 => Input::new(1.0, 0.6, true, false, false),
        // Hold jump through this phase: exercises the airborne path in the hash.
        241..=360 => Input::new(1.0, -0.8, false, false, true),
        361..=420 => Input::new(-1.0, 0.3, false, true, false),
        _ => Input::new(0.5, 0.0, false, false, false),
    }
}

fn run(ticks: u32) -> u64 {
    let mut world = World::new();
    for tick in 0..ticks {
        world.step(&scripted(tick));
    }
    world.state_hash()
}

#[test]
fn two_identical_runs_agree() {
    assert_eq!(run(4000), run(4000));
}

#[test]
fn runs_across_threads_agree() {
    // Any hidden shared mutable state, iteration-order dependence or thread
    // local randomness would show up here.
    let handles: Vec<_> = (0..8).map(|_| std::thread::spawn(|| run(3000))).collect();
    let hashes: Vec<u64> = handles.into_iter().map(|h| h.join().unwrap()).collect();
    assert!(
        hashes.windows(2).all(|w| w[0] == w[1]),
        "threads diverged: {hashes:?}"
    );
}

#[test]
fn hash_is_sensitive_to_input() {
    // Sanity check on the hash itself: it must actually detect divergence.
    let mut a = World::new();
    let mut b = World::new();
    for tick in 0..600 {
        a.step(&scripted(tick));
        // One tick of subtly different steering, on a tick where the car is on
        // the ground (steering does nothing in the air, so a change there
        // correctly has no effect).
        let mut alt = scripted(tick);
        if tick == 100 {
            alt.steer += 1e-9;
        }
        b.step(&alt);
    }
    assert_ne!(a.state_hash(), b.state_hash());
}

#[test]
fn step_has_no_hidden_history_dependence() {
    // Snapshot a warmed-up world, branch it, and replay the same inputs down
    // both branches. Relevant for rollback netcode: state + inputs must fully
    // determine the future, with nothing else leaking in.
    let mut world = World::new();
    for tick in 0..1500 {
        world.step(&scripted(tick));
    }

    let mut a = world.clone();
    let mut b = world.clone();
    for tick in 1500..2100 {
        a.step(&scripted(tick));
        b.step(&scripted(tick));
    }
    assert_eq!(a.state_hash(), b.state_hash());
    assert_eq!(a.snapshot().ball_pos, b.snapshot().ball_pos);
}

#[test]
fn tick_rate_is_fixed() {
    // The simulation contract: 120 Hz, and DT is exactly its reciprocal.
    assert_eq!(TICK_HZ, 120);
    assert_eq!(rl_sim::DT, 1.0 / 120.0);
}
