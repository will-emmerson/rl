//! Keyboard and gamepad bindings.
//!
//! The defaults mirror the viewer's historical controls (and Rocket League's
//! pad layout). They are loaded from `controls.toml` at the repo root, or from
//! `--controls-config PATH`, and hot-reloaded while the viewer runs - the same
//! treatment `camera.toml` gets.
//!
//! Bindings name physical controls. Keyboard names accept both Bevy's `KeyCode`
//! spellings (`KeyW`, `ArrowUp`, `ControlLeft`) and familiar short ones (`W`,
//! `Up`, `Ctrl`). A binding may be one string or a list, so an action can live
//! on several keys. Gamepad buttons use Bevy/gilrs' names (`South`, `South` is
//! the Xbox A / PlayStation Cross button), with common aliases such as `A`,
//! `RT` and `R3`; if a pad's labels disagree with what actually happens, run the
//! viewer with `--gamepad-debug` and bind the name it logs.

use std::fmt;

use bevy::input::gamepad::{Gamepad, GamepadAxis, GamepadButton};
use bevy::input::keyboard::KeyCode;
use bevy::prelude::*;
use serde::de::{self, Deserializer, SeqAccess, Visitor};
use serde::Deserialize;

/// Every binding the viewer knows about.
#[derive(Resource, Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Controls {
    pub keyboard: KeyboardControls,
    pub gamepad: GamepadControls,
}

/// Keyboard bindings. Each direction/action takes one key name or a list.
///
/// `pitch_forward` is a frontflip (nose down) and `pitch_backward` a backflip,
/// matching the way Rocket League's stick works. The defaults duplicate the
/// drive keys so the same keys steer on the ground and rotate in the air; bind
/// them separately to separate the two.
#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct KeyboardControls {
    pub throttle_forward: KeyList,
    pub throttle_backward: KeyList,
    pub steer_left: KeyList,
    pub steer_right: KeyList,
    pub pitch_forward: KeyList,
    pub pitch_backward: KeyList,
    pub yaw_left: KeyList,
    pub yaw_right: KeyList,
    pub roll_left: KeyList,
    pub roll_right: KeyList,
    pub jump: KeyList,
    pub boost: KeyList,
    pub drift: KeyList,
    pub look_left: KeyList,
    pub look_right: KeyList,
    pub look_up: KeyList,
    pub look_down: KeyList,
    pub look_behind: KeyList,
    pub toggle_camera: KeyList,
    pub reset: KeyList,
    pub quit: KeyList,
}

impl Default for KeyboardControls {
    fn default() -> Self {
        use KeyCode::*;
        Self {
            throttle_forward: KeyList::new(&[KeyW, ArrowUp]),
            throttle_backward: KeyList::new(&[KeyS, ArrowDown]),
            steer_left: KeyList::new(&[KeyA, ArrowLeft]),
            steer_right: KeyList::new(&[KeyD, ArrowRight]),
            // The drive keys double as air control, as in Rocket League's
            // keyboard preset: forward is a frontflip, back a backflip.
            pitch_forward: KeyList::new(&[KeyW, ArrowUp]),
            pitch_backward: KeyList::new(&[KeyS, ArrowDown]),
            yaw_left: KeyList::new(&[KeyA, ArrowLeft]),
            yaw_right: KeyList::new(&[KeyD, ArrowRight]),
            roll_left: KeyList::new(&[KeyQ]),
            roll_right: KeyList::new(&[KeyE]),
            jump: KeyList::new(&[ControlLeft, ControlRight]),
            boost: KeyList::new(&[Space]),
            drift: KeyList::new(&[ShiftLeft, ShiftRight]),
            look_left: KeyList::new(&[KeyJ]),
            look_right: KeyList::new(&[KeyL]),
            look_up: KeyList::new(&[KeyI]),
            look_down: KeyList::new(&[KeyK]),
            // No keyboard look-behind by default, as before; bind it if wanted.
            look_behind: KeyList::new(&[]),
            toggle_camera: KeyList::new(&[KeyC]),
            reset: KeyList::new(&[KeyR]),
            quit: KeyList::new(&[Escape]),
        }
    }
}

/// Gamepad bindings.
///
/// Movement is shown as one steer axis and one pitch axis, matching the physical
/// sticks; triggers are separate analogue throttle bindings. Digital actions
/// take one button name or a list.
#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GamepadControls {
    pub steer_axis: AxisBinding,
    pub pitch_axis: AxisBinding,
    pub look_x_axis: AxisBinding,
    pub look_y_axis: AxisBinding,
    pub throttle_forward: Trigger,
    pub throttle_backward: Trigger,
    pub jump: ButtonList,
    pub boost: ButtonList,
    pub drift: ButtonList,
    /// Dedicated air-roll buttons, *in addition to* RL's drift + stick roll.
    /// Empty by default; bind e.g. `roll_left = "North"` (X) if wanted.
    pub roll_left: ButtonList,
    pub roll_right: ButtonList,
    pub look_behind: ButtonList,
    pub toggle_camera: ButtonList,
    pub reset: ButtonList,
    pub quit: ButtonList,
}

impl Default for GamepadControls {
    fn default() -> Self {
        Self {
            steer_axis: AxisBinding(GamepadAxis::LeftStickX),
            pitch_axis: AxisBinding(GamepadAxis::LeftStickY),
            look_x_axis: AxisBinding(GamepadAxis::RightStickX),
            look_y_axis: AxisBinding(GamepadAxis::RightStickY),
            // Most pads report the analogue triggers as the `LeftZ`/`RightZ`
            // axes; accept the trigger buttons too, for pads that report them
            // that way.
            throttle_forward: Trigger {
                axis: Some(AxisBinding(GamepadAxis::RightZ)),
                button: Some(PadButton(GamepadButton::RightTrigger2)),
            },
            throttle_backward: Trigger {
                axis: Some(AxisBinding(GamepadAxis::LeftZ)),
                button: Some(PadButton(GamepadButton::LeftTrigger2)),
            },
            jump: ButtonList::new(&[GamepadButton::South]),
            boost: ButtonList::new(&[GamepadButton::East]),
            drift: ButtonList::new(&[GamepadButton::North]),
            // RL's default pad layout has no dedicated air-roll buttons; the
            // drift button plus the stick rolls. Users can add X/B or bumpers.
            roll_left: ButtonList::new(&[]),
            roll_right: ButtonList::new(&[]),
            look_behind: ButtonList::new(&[GamepadButton::RightThumb]),
            toggle_camera: ButtonList::new(&[GamepadButton::West]),
            reset: ButtonList::new(&[GamepadButton::Start]),
            // No default quit button: Start resets, and a stray press should
            // not close the game.
            quit: ButtonList::new(&[]),
        }
    }
}

/// One or more keyboard keys.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct KeyList(Vec<KeyCode>);

impl KeyList {
    pub fn new(keys: &[KeyCode]) -> Self {
        Self(keys.to_vec())
    }

    pub fn pressed(&self, keys: &ButtonInput<KeyCode>) -> bool {
        self.0.iter().any(|key| keys.pressed(*key))
    }

    pub fn just_pressed(&self, keys: &ButtonInput<KeyCode>) -> bool {
        self.0.iter().any(|key| keys.just_pressed(*key))
    }
}

impl<'de> Deserialize<'de> for KeyList {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(KeyListVisitor)
    }
}

struct KeyListVisitor;

impl<'de> Visitor<'de> for KeyListVisitor {
    type Value = KeyList;

    fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str("a key name or a list of key names")
    }

    fn visit_str<E: de::Error>(self, value: &str) -> Result<Self::Value, E> {
        parse_key(value)
            .map(|key| KeyList(vec![key]))
            .ok_or_else(|| E::custom(unknown_key(value)))
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
        let mut keys = Vec::new();
        while let Some(value) = seq.next_element::<String>()? {
            let key = parse_key(&value).ok_or_else(|| de::Error::custom(unknown_key(&value)))?;
            keys.push(key);
        }
        Ok(KeyList(keys))
    }
}

/// One or more gamepad buttons.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ButtonList(Vec<GamepadButton>);

impl ButtonList {
    pub fn new(buttons: &[GamepadButton]) -> Self {
        Self(buttons.to_vec())
    }

    pub fn pressed(&self, pad: &Gamepad) -> bool {
        self.0.iter().any(|button| pad.pressed(*button))
    }

    pub fn just_pressed(&self, pad: &Gamepad) -> bool {
        self.0.iter().any(|button| pad.just_pressed(*button))
    }
}

impl<'de> Deserialize<'de> for ButtonList {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(ButtonListVisitor)
    }
}

struct ButtonListVisitor;

impl<'de> Visitor<'de> for ButtonListVisitor {
    type Value = ButtonList;

    fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str("a gamepad button name or a list of button names")
    }

    fn visit_str<E: de::Error>(self, value: &str) -> Result<Self::Value, E> {
        parse_button(value)
            .map(|button| ButtonList(vec![button]))
            .ok_or_else(|| E::custom(unknown_button(value)))
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
        let mut buttons = Vec::new();
        while let Some(value) = seq.next_element::<String>()? {
            let button =
                parse_button(&value).ok_or_else(|| de::Error::custom(unknown_button(&value)))?;
            buttons.push(button);
        }
        Ok(ButtonList(buttons))
    }
}

/// A gamepad axis, named by its `GamepadAxis` variant or a short alias.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisBinding(pub GamepadAxis);

impl<'de> Deserialize<'de> for AxisBinding {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = String::deserialize(deserializer)?;
        parse_axis(&value)
            .map(AxisBinding)
            .ok_or_else(|| de::Error::custom(unknown_axis(&value)))
    }
}

/// A gamepad button, named by its `GamepadButton` variant or a short alias.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PadButton(pub GamepadButton);

impl<'de> Deserialize<'de> for PadButton {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = String::deserialize(deserializer)?;
        parse_button(&value)
            .map(PadButton)
            .ok_or_else(|| de::Error::custom(unknown_button(&value)))
    }
}

/// An analogue trigger, in `0.0..=1.0`.
///
/// gilrs maps a pad through SDL's controller database. A pad with no entry (see
/// `examples/gamepad_probe.rs`) falls back to gilrs' built-in mapping, which
/// exposes the xpad triggers as the *axes* `LeftZ`/`RightZ` (evdev ABS_Z/ABS_RZ).
/// Those axes run -1.0..=1.0 and rest near -1.0, reaching +1.0 when fully
/// pulled, so the axis reading is normalised here. Pads that *do* have a
/// database entry report the triggers as buttons instead; accept either and use
/// the stronger reading. An empty table disables that direction.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Trigger {
    axis: Option<AxisBinding>,
    button: Option<PadButton>,
}

impl Trigger {
    /// The axis this trigger reads, if configured.
    pub fn axis(self) -> Option<GamepadAxis> {
        self.axis.map(|axis| axis.0)
    }

    pub fn value(self, pad: &Gamepad) -> f64 {
        let from_axis = self
            .axis
            .and_then(|axis| pad.get(axis.0))
            .map(|value| ((value as f64 + 1.0) * 0.5).clamp(0.0, 1.0))
            .unwrap_or(0.0);
        let from_button = self
            .button
            .and_then(|button| pad.get(button.0))
            .unwrap_or(0.0) as f64;

        // Axis and button readings both rest at (about) zero, but a trigger axis
        // at rest can map to a small non-zero value; trim the slack so the car
        // does not creep at idle.
        let value = strongest(from_axis, from_button);
        if value < 0.05 {
            0.0
        } else {
            value
        }
    }
}

/// Whichever of the two is further from zero, so keyboard and pad can be used
/// interchangeably without cancelling each other out.
pub fn strongest(a: f64, b: f64) -> f64 {
    if a.abs() >= b.abs() {
        a
    } else {
        b
    }
}

/// Seed a pad's configured trigger axes to their resting value.
///
/// Bevy pre-fills every gamepad axis with `0.0` until the pad sends its first
/// event, but a trigger axis does not rest there: `LeftZ`/`RightZ` rest at
/// `-1.0`, so `0.0` normalises to `0.5` (half-pressed). After the player uses
/// one trigger, that one reads `0.0` while its untouched partner still reads
/// `0.5`, and the difference looks like reverse throttle as soon as the trigger
/// is released. Seeding the untouched axes to `-1.0` keeps both at rest.
///
/// An axis that has already reported a value is left alone, so this is safe to
/// call whenever a pad appears or the bindings are reloaded.
pub fn seed_trigger_axes(pad: &mut Gamepad, controls: &Controls) {
    for trigger in [
        &controls.gamepad.throttle_forward,
        &controls.gamepad.throttle_backward,
    ] {
        if let Some(axis) = trigger.axis() {
            if pad.get(axis).is_none_or(|value| value == 0.0) {
                pad.analog_mut().set(axis, -1.0);
            }
        }
    }
}

/// A signed axis from two sets of keys: +1 when only `positive` is held, -1 when
/// only `negative` is.
pub fn key_axis(keys: &ButtonInput<KeyCode>, positive: &KeyList, negative: &KeyList) -> f64 {
    match (positive.pressed(keys), negative.pressed(keys)) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}

/// A signed axis from two sets of pad buttons: +1 when only `positive` is held,
/// -1 when only `negative` is.
pub fn button_axis(pad: &Gamepad, positive: &ButtonList, negative: &ButtonList) -> f64 {
    match (positive.pressed(pad), negative.pressed(pad)) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}

fn normalized(name: &str) -> String {
    name.chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| c.to_ascii_lowercase())
        .collect()
}

fn unknown_key(value: &str) -> String {
    format!(
        "unknown key {value:?}; use a KeyCode name (KeyW, ArrowUp, ControlLeft) \
         or a short name (W, Up, Ctrl). If {value:?} is a gamepad button such as \
         LB or RB, put it in the [gamepad] section instead."
    )
}

fn unknown_button(value: &str) -> String {
    format!(
        "unknown gamepad button {value:?}; use a GamepadButton name (South, \
         RightTrigger, RightThumb, DPadUp) or an alias (A, LB, RB, LT, RT, R3). \
         If {value:?} is a keyboard key, put it in the [keyboard] section instead."
    )
}

fn unknown_axis(value: &str) -> String {
    format!(
        "unknown gamepad axis {value:?}; use LeftStickX, LeftStickY, LeftZ, \
         RightStickX, RightStickY or RightZ"
    )
}

/// Parse a keyboard key name. Both `KeyCode` spellings and friendly aliases are
/// accepted, case-insensitively and ignoring `_`, `-` and spaces.
pub fn parse_key(name: &str) -> Option<KeyCode> {
    let n = normalized(name);

    // Letters and digits, with or without Bevy's `Key`/`Digit` prefix: `w`,
    // `KeyW`, `key_w`, `1`, `Digit1`.
    let bare = n.strip_prefix("key").unwrap_or(&n);
    if let Some(c) = single_char(bare) {
        if let Some(key) = letter_key(c) {
            return Some(key);
        }
        if let Some(key) = digit_key(c) {
            return Some(key);
        }
    }
    if let Some(digit) = n.strip_prefix("digit").and_then(single_char) {
        if let Some(key) = digit_key(digit) {
            return Some(key);
        }
    }
    if let Some(digit) = n.strip_prefix("numpad").and_then(single_char) {
        if let Some(key) = numpad_key(digit) {
            return Some(key);
        }
    }
    if let Some(number) = n.strip_prefix('f').and_then(|s| s.parse::<u8>().ok()) {
        if let Some(key) = function_key(number) {
            return Some(key);
        }
    }

    let key = match n.as_str() {
        "up" | "arrowup" => KeyCode::ArrowUp,
        "down" | "arrowdown" => KeyCode::ArrowDown,
        "left" | "arrowleft" => KeyCode::ArrowLeft,
        "right" | "arrowright" => KeyCode::ArrowRight,
        "space" | "spacebar" => KeyCode::Space,
        "enter" | "return" => KeyCode::Enter,
        "escape" | "esc" => KeyCode::Escape,
        "tab" => KeyCode::Tab,
        "backspace" => KeyCode::Backspace,
        "delete" | "del" => KeyCode::Delete,
        "insert" | "ins" => KeyCode::Insert,
        "home" => KeyCode::Home,
        "end" => KeyCode::End,
        "pageup" | "pgup" => KeyCode::PageUp,
        "pagedown" | "pgdn" => KeyCode::PageDown,
        "shift" | "shiftleft" | "leftshift" => KeyCode::ShiftLeft,
        "shiftright" | "rightshift" => KeyCode::ShiftRight,
        "ctrl" | "control" | "controlleft" | "leftctrl" | "leftcontrol" => KeyCode::ControlLeft,
        "ctrlright" | "controlright" | "rightctrl" | "rightcontrol" => KeyCode::ControlRight,
        "alt" | "altleft" | "leftalt" => KeyCode::AltLeft,
        "altright" | "rightalt" => KeyCode::AltRight,
        "super" | "meta" | "superleft" | "leftsuper" | "leftmeta" | "cmd" | "command" => {
            KeyCode::SuperLeft
        }
        "superright" | "rightsuper" | "rightmeta" => KeyCode::SuperRight,
        "capslock" => KeyCode::CapsLock,
        "contextmenu" | "menu" => KeyCode::ContextMenu,
        "backquote" | "grave" | "tilde" => KeyCode::Backquote,
        "minus" => KeyCode::Minus,
        "equal" | "equals" => KeyCode::Equal,
        "bracketleft" | "leftbracket" => KeyCode::BracketLeft,
        "bracketright" | "rightbracket" => KeyCode::BracketRight,
        "backslash" => KeyCode::Backslash,
        "intlbackslash" => KeyCode::IntlBackslash,
        "semicolon" => KeyCode::Semicolon,
        "quote" | "apostrophe" => KeyCode::Quote,
        "comma" => KeyCode::Comma,
        "period" | "dot" | "fullstop" => KeyCode::Period,
        "slash" | "forwardslash" => KeyCode::Slash,
        "numlock" => KeyCode::NumLock,
        "numpadadd" | "numpadplus" => KeyCode::NumpadAdd,
        "numpadsubtract" | "numpadminus" => KeyCode::NumpadSubtract,
        "numpadmultiply" | "numpadstar" => KeyCode::NumpadMultiply,
        "numpaddivide" => KeyCode::NumpadDivide,
        "numpadenter" => KeyCode::NumpadEnter,
        "numpaddecimal" => KeyCode::NumpadDecimal,
        "numpadcomma" => KeyCode::NumpadComma,
        "numpadequal" => KeyCode::NumpadEqual,
        "printscreen" | "prtsc" => KeyCode::PrintScreen,
        "scrolllock" => KeyCode::ScrollLock,
        "pause" => KeyCode::Pause,
        _ => return None,
    };
    Some(key)
}

/// Parse a gamepad button name. Accepts `GamepadButton` spellings and common
/// positional aliases (Xbox/PlayStation labels).
///
/// In particular the bumpers are `LB`/`L1`/`LeftTrigger` and
/// `RB`/`R1`/`RightTrigger`, while the analogue triggers are
/// `LT`/`L2`/`LeftTrigger2` and `RT`/`R2`/`RightTrigger2` - the `2` suffix is
/// gilrs' second-trigger slot, not a second bumper.
pub fn parse_button(name: &str) -> Option<GamepadButton> {
    let n = normalized(name);
    let button = match n.as_str() {
        "south" | "a" | "cross" => GamepadButton::South,
        "east" | "b" | "circle" => GamepadButton::East,
        "north" | "x" | "triangle" => GamepadButton::North,
        "west" | "y" | "square" => GamepadButton::West,
        "c" => GamepadButton::C,
        "z" => GamepadButton::Z,
        "lefttrigger" | "lb" | "l1" | "leftbumper" | "leftshoulder" => GamepadButton::LeftTrigger,
        "lefttrigger2" | "lt" | "l2" => GamepadButton::LeftTrigger2,
        "righttrigger" | "rb" | "r1" | "rightbumper" | "rightshoulder" => {
            GamepadButton::RightTrigger
        }
        "righttrigger2" | "rt" | "r2" => GamepadButton::RightTrigger2,
        "select" | "back" | "view" | "share" => GamepadButton::Select,
        "start" | "menu" | "options" => GamepadButton::Start,
        "mode" | "guide" | "home" | "ps" => GamepadButton::Mode,
        "leftthumb" | "l3" | "leftstickclick" => GamepadButton::LeftThumb,
        "rightthumb" | "r3" | "rightstickclick" => GamepadButton::RightThumb,
        "dpadup" | "up" => GamepadButton::DPadUp,
        "dpaddown" | "down" => GamepadButton::DPadDown,
        "dpadleft" | "left" => GamepadButton::DPadLeft,
        "dpadright" | "right" => GamepadButton::DPadRight,
        _ => {
            let index = n.strip_prefix("other")?.parse::<u8>().ok()?;
            return Some(GamepadButton::Other(index));
        }
    };
    Some(button)
}

/// Parse a gamepad axis name. Accepts `GamepadAxis` spellings and short aliases.
pub fn parse_axis(name: &str) -> Option<GamepadAxis> {
    let n = normalized(name);
    let axis = match n.as_str() {
        "leftstickx" | "leftx" | "lx" => GamepadAxis::LeftStickX,
        "leftsticky" | "lefty" | "ly" => GamepadAxis::LeftStickY,
        "leftz" | "lz" | "lefttriggeraxis" => GamepadAxis::LeftZ,
        "rightstickx" | "rightx" | "rx" => GamepadAxis::RightStickX,
        "rightsticky" | "righty" | "ry" => GamepadAxis::RightStickY,
        "rightz" | "rz" | "righttriggeraxis" => GamepadAxis::RightZ,
        _ => {
            let index = n.strip_prefix("other")?.parse::<u8>().ok()?;
            return Some(GamepadAxis::Other(index));
        }
    };
    Some(axis)
}

fn single_char(value: &str) -> Option<char> {
    let mut chars = value.chars();
    let c = chars.next()?;
    chars.next().is_none().then_some(c)
}

macro_rules! key_match {
    ($name:ident, $value:ty, $($text:literal => $variant:ident),* $(,)?) => {
        fn $name(value: $value) -> Option<KeyCode> {
            match value {
                $($text => Some(KeyCode::$variant),)*
                _ => None,
            }
        }
    };
}

key_match!(letter_key, char,
    'a' => KeyA, 'b' => KeyB, 'c' => KeyC, 'd' => KeyD, 'e' => KeyE,
    'f' => KeyF, 'g' => KeyG, 'h' => KeyH, 'i' => KeyI, 'j' => KeyJ,
    'k' => KeyK, 'l' => KeyL, 'm' => KeyM, 'n' => KeyN, 'o' => KeyO,
    'p' => KeyP, 'q' => KeyQ, 'r' => KeyR, 's' => KeyS, 't' => KeyT,
    'u' => KeyU, 'v' => KeyV, 'w' => KeyW, 'x' => KeyX, 'y' => KeyY,
    'z' => KeyZ,
);

key_match!(digit_key, char,
    '0' => Digit0, '1' => Digit1, '2' => Digit2, '3' => Digit3, '4' => Digit4,
    '5' => Digit5, '6' => Digit6, '7' => Digit7, '8' => Digit8, '9' => Digit9,
);

key_match!(numpad_key, char,
    '0' => Numpad0, '1' => Numpad1, '2' => Numpad2, '3' => Numpad3, '4' => Numpad4,
    '5' => Numpad5, '6' => Numpad6, '7' => Numpad7, '8' => Numpad8, '9' => Numpad9,
);

fn function_key(number: u8) -> Option<KeyCode> {
    let key = match number {
        1 => KeyCode::F1,
        2 => KeyCode::F2,
        3 => KeyCode::F3,
        4 => KeyCode::F4,
        5 => KeyCode::F5,
        6 => KeyCode::F6,
        7 => KeyCode::F7,
        8 => KeyCode::F8,
        9 => KeyCode::F9,
        10 => KeyCode::F10,
        11 => KeyCode::F11,
        12 => KeyCode::F12,
        13 => KeyCode::F13,
        14 => KeyCode::F14,
        15 => KeyCode::F15,
        16 => KeyCode::F16,
        17 => KeyCode::F17,
        _ => return None,
    };
    Some(key)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_the_documented_controls() {
        let controls = Controls::default();
        let keys = &controls.keyboard;
        assert_eq!(
            keys.throttle_forward,
            KeyList::new(&[KeyCode::KeyW, KeyCode::ArrowUp])
        );
        assert_eq!(
            keys.steer_left,
            KeyList::new(&[KeyCode::KeyA, KeyCode::ArrowLeft])
        );
        assert_eq!(keys.roll_left, KeyList::new(&[KeyCode::KeyQ]));
        assert_eq!(keys.roll_right, KeyList::new(&[KeyCode::KeyE]));
        assert_eq!(
            keys.jump,
            KeyList::new(&[KeyCode::ControlLeft, KeyCode::ControlRight])
        );
        assert_eq!(keys.boost, KeyList::new(&[KeyCode::Space]));
        assert_eq!(
            keys.drift,
            KeyList::new(&[KeyCode::ShiftLeft, KeyCode::ShiftRight])
        );
        assert_eq!(keys.toggle_camera, KeyList::new(&[KeyCode::KeyC]));
        assert_eq!(keys.reset, KeyList::new(&[KeyCode::KeyR]));
        assert_eq!(keys.quit, KeyList::new(&[KeyCode::Escape]));

        let pad = &controls.gamepad;
        assert_eq!(pad.jump, ButtonList::new(&[GamepadButton::South]));
        assert_eq!(pad.boost, ButtonList::new(&[GamepadButton::East]));
        assert_eq!(pad.drift, ButtonList::new(&[GamepadButton::North]));
        assert!(pad.roll_left.0.is_empty());
        assert!(pad.roll_right.0.is_empty());
        assert_eq!(pad.toggle_camera, ButtonList::new(&[GamepadButton::West]));
        assert_eq!(pad.reset, ButtonList::new(&[GamepadButton::Start]));
        assert_eq!(
            pad.look_behind,
            ButtonList::new(&[GamepadButton::RightThumb])
        );
        assert_eq!(pad.steer_axis, AxisBinding(GamepadAxis::LeftStickX));
        assert_eq!(pad.look_x_axis, AxisBinding(GamepadAxis::RightStickX));
    }

    #[test]
    fn keys_accept_bevy_names_and_short_aliases() {
        assert_eq!(parse_key("KeyW"), Some(KeyCode::KeyW));
        assert_eq!(parse_key("w"), Some(KeyCode::KeyW));
        assert_eq!(parse_key("up"), Some(KeyCode::ArrowUp));
        assert_eq!(parse_key("ArrowUp"), Some(KeyCode::ArrowUp));
        assert_eq!(parse_key("ControlLeft"), Some(KeyCode::ControlLeft));
        assert_eq!(parse_key("ctrl"), Some(KeyCode::ControlLeft));
        assert_eq!(parse_key("ShiftLeft"), Some(KeyCode::ShiftLeft));
        assert_eq!(parse_key("space"), Some(KeyCode::Space));
        assert_eq!(parse_key("Digit1"), Some(KeyCode::Digit1));
        assert_eq!(parse_key("1"), Some(KeyCode::Digit1));
        assert_eq!(parse_key("Numpad7"), Some(KeyCode::Numpad7));
        assert_eq!(parse_key("f11"), Some(KeyCode::F11));
        assert_eq!(parse_key("left ctrl"), Some(KeyCode::ControlLeft));
        assert_eq!(parse_key("not-a-key"), None);
    }

    #[test]
    fn buttons_and_axes_accept_names_and_aliases() {
        assert_eq!(parse_button("South"), Some(GamepadButton::South));
        assert_eq!(parse_button("a"), Some(GamepadButton::South));
        assert_eq!(parse_button("X"), Some(GamepadButton::North));
        assert_eq!(parse_button("rt"), Some(GamepadButton::RightTrigger2));
        assert_eq!(parse_button("R3"), Some(GamepadButton::RightThumb));
        assert_eq!(parse_button("D-Pad Left"), Some(GamepadButton::DPadLeft));
        assert_eq!(parse_button("Other7"), Some(GamepadButton::Other(7)));
        assert_eq!(parse_button("nope"), None);

        // The bumpers are the `Trigger` buttons and the analogue triggers are
        // the `Trigger2` slots. This is the naming gilrs uses and the source of
        // most confusion between LB/RB and LT/RT.
        for name in ["LB", "L1", "LeftTrigger", "left bumper", "left shoulder"] {
            assert_eq!(
                parse_button(name),
                Some(GamepadButton::LeftTrigger),
                "{name}"
            );
        }
        for name in ["RB", "R1", "RightTrigger", "right bumper", "right shoulder"] {
            assert_eq!(
                parse_button(name),
                Some(GamepadButton::RightTrigger),
                "{name}"
            );
        }
        for name in ["LT", "L2", "LeftTrigger2"] {
            assert_eq!(
                parse_button(name),
                Some(GamepadButton::LeftTrigger2),
                "{name}"
            );
        }
        for name in ["RT", "R2", "RightTrigger2"] {
            assert_eq!(
                parse_button(name),
                Some(GamepadButton::RightTrigger2),
                "{name}"
            );
        }

        assert_eq!(parse_axis("LeftStickX"), Some(GamepadAxis::LeftStickX));
        assert_eq!(parse_axis("lx"), Some(GamepadAxis::LeftStickX));
        assert_eq!(parse_axis("right_z"), Some(GamepadAxis::RightZ));
        assert_eq!(parse_axis("Other2"), Some(GamepadAxis::Other(2)));
        assert_eq!(parse_axis("nope"), None);
    }

    #[test]
    fn a_partial_controls_file_keeps_the_other_defaults() {
        let error = toml::from_str::<Controls>(
            "[keyboard]\n\
             boost = [\"F\", \"Mouse1\"]\n\
             roll_left = \"Z\"\n\
             \n\
             [gamepad]\n\
             jump = \"North\"\n\
             look_x_axis = \"rx\"\n",
        )
        .unwrap_err();
        // "Mouse1" is not a key, so the file is rejected rather than silently
        // dropping one binding.
        assert!(error.to_string().contains("Mouse1"), "{error}");

        let controls: Controls = toml::from_str(
            "[keyboard]\n\
             boost = [\"F\"]\n\
             roll_left = \"Z\"\n\
             \n\
             [gamepad]\n\
             jump = \"North\"\n\
             look_x_axis = \"rx\"\n",
        )
        .unwrap();
        assert_eq!(controls.keyboard.boost, KeyList::new(&[KeyCode::KeyF]));
        assert_eq!(controls.keyboard.roll_left, KeyList::new(&[KeyCode::KeyZ]));
        // Untouched fields come from the defaults.
        assert_eq!(controls.keyboard.reset, KeyList::new(&[KeyCode::KeyR]));
        assert_eq!(
            controls.gamepad.jump,
            ButtonList::new(&[GamepadButton::North])
        );
        assert_eq!(
            controls.gamepad.look_x_axis,
            AxisBinding(GamepadAxis::RightStickX)
        );
        assert_eq!(
            controls.gamepad.throttle_forward,
            GamepadControls::default().throttle_forward
        );
    }

    #[test]
    fn an_empty_binding_unbinds_an_action() {
        let controls: Controls =
            toml::from_str("[keyboard]\nquit = []\n[gamepad]\nquit = []\n").unwrap();
        assert!(!controls.keyboard.quit.pressed(&ButtonInput::default()));
        assert!(controls.gamepad.quit.0.is_empty());
    }

    #[test]
    fn unknown_sections_are_rejected() {
        assert!(toml::from_str::<Controls>("[keybord]\nboost = \"F\"").is_err());
    }

    #[test]
    fn a_pad_button_in_the_keyboard_section_says_where_it_belongs() {
        let error = toml::from_str::<Controls>("[keyboard]\nboost = \"RB\"")
            .unwrap_err()
            .to_string();
        assert!(error.contains("unknown key"), "{error}");
        assert!(error.contains("[gamepad]"), "{error}");
    }

    #[test]
    fn seeding_trigger_axes_sets_the_rest_value() {
        let mut pad = Gamepad::default();
        seed_trigger_axes(&mut pad, &Controls::default());
        assert_eq!(pad.get(GamepadAxis::LeftZ), Some(-1.0));
        assert_eq!(pad.get(GamepadAxis::RightZ), Some(-1.0));

        // An axis that has already reported a value is left alone.
        pad.analog_mut().set(GamepadAxis::RightZ, 0.5);
        seed_trigger_axes(&mut pad, &Controls::default());
        assert_eq!(pad.get(GamepadAxis::RightZ), Some(0.5));
    }

    #[test]
    fn key_axis_reads_a_direction_pair() {
        let positive = KeyList::new(&[KeyCode::KeyA]);
        let negative = KeyList::new(&[KeyCode::KeyD]);
        let mut keys = ButtonInput::<KeyCode>::default();
        assert_eq!(key_axis(&keys, &positive, &negative), 0.0);
        keys.press(KeyCode::KeyA);
        assert_eq!(key_axis(&keys, &positive, &negative), 1.0);
        keys.press(KeyCode::KeyD);
        assert_eq!(key_axis(&keys, &positive, &negative), 0.0);
        keys.release(KeyCode::KeyA);
        assert_eq!(key_axis(&keys, &positive, &negative), -1.0);
    }
}
