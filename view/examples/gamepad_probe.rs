//! Prints how gilrs maps each connected gamepad - no button presses needed.
//!
//!     cargo run -p rl_view --example gamepad_probe
//!
//! This is the diagnostic to reach for when a pad is detected but "the buttons
//! are wrong". It reports which mapping gilrs picked (an entry from the SDL
//! controller database, or the built-in driver default) and what each button and
//! axis is called *after* mapping - which is exactly what lands in Bevy as
//! `GamepadButton` / `GamepadAxis`.
//!
//! Note that gilrs' built-in default is per-device: it starts from native evdev
//! codes and drops anything the pad doesn't have. For an xpad-style X-Box pad
//! that means the analogue triggers show up as the axes `LeftZ` / `RightZ`,
//! because that is what ABS_Z / ABS_RZ are called, *not* as trigger buttons.

use std::time::{Duration, Instant};

use gilrs::{Axis, Button, Gilrs, MappingSource};

const BUTTONS: [Button; 19] = [
    Button::South,
    Button::East,
    Button::North,
    Button::West,
    Button::LeftTrigger,
    Button::RightTrigger,
    Button::LeftTrigger2,
    Button::RightTrigger2,
    Button::Select,
    Button::Start,
    Button::Mode,
    Button::LeftThumb,
    Button::RightThumb,
    Button::DPadUp,
    Button::DPadDown,
    Button::DPadLeft,
    Button::DPadRight,
    Button::C,
    Button::Z,
];

const AXES: [Axis; 8] = [
    Axis::LeftStickX,
    Axis::LeftStickY,
    Axis::LeftZ,
    Axis::RightStickX,
    Axis::RightStickY,
    Axis::RightZ,
    Axis::DPadX,
    Axis::DPadY,
];

fn main() {
    let mut gilrs = Gilrs::new().expect("failed to start gilrs");

    // Pads already present are enumerated by gilrs' constructor, but pumping a
    // few events for a moment is a cheap way to be sure. Bounded so this can
    // never hang the shell it is run from.
    let deadline = Instant::now() + Duration::from_millis(600);
    while Instant::now() < deadline {
        while gilrs.next_event().is_some() {}
        std::thread::sleep(Duration::from_millis(10));
    }

    let mut found = false;
    for (id, pad) in gilrs.gamepads() {
        found = true;
        println!("gamepad {id}: {}", pad.name());

        let source = match pad.mapping_source() {
            MappingSource::SdlMappings => "SDL database entry",
            MappingSource::Driver => "gilrs built-in default (no SDL entry for this pad)",
            MappingSource::None => "none - events will be raw/Unknown",
        };
        println!("  mapping: {source}");

        let buttons: Vec<String> = BUTTONS
            .into_iter()
            .filter_map(|b| pad.button_code(b).map(|c| format!("    {b:?} <- {c:?}")))
            .collect();
        println!("  buttons ({}):", buttons.len());
        for line in &buttons {
            println!("{line}");
        }

        let axes: Vec<String> = AXES
            .into_iter()
            .filter_map(|a| {
                pad.axis_code(a)
                    .map(|c| format!("    {a:?} <- {c:?}  value={:+.3}", pad.value(a)))
            })
            .collect();
        println!("  axes ({}):", axes.len());
        for line in &axes {
            println!("{line}");
        }
    }

    if !found {
        println!("no gamepad connected");
        return;
    }

    println!();
    println!("values are gilrs' cached state - a pad untouched since startup reads 0.000.");
    println!("watch them change with:  cargo run -p rl_view -- --gamepad-debug");
}
