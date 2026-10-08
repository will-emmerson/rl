//! Terminal front end for `rl_sim`.
//!
//! Two modes:
//!   * scripted (default) — deterministic input, so the state hash printed at
//!     the end is a reproducibility check, not an accident.
//!   * interactive (`--interactive`) — drive with WASD, needs a TTY.
//!
//! The renderer only ever *reads* world state. Nothing in the draw path feeds
//! back into the simulation, which is what keeps the two modes comparable.

use std::io::{self, IsTerminal, Read, Write};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use rl_sim::car::CAR_HALF_LENGTH;
use rl_sim::{Input, World, DT, TICK_HZ};

const COLS: usize = 40;
const ROWS: usize = 18;

/// How long a key counts as held after the last byte we saw for it, in ticks.
/// Raw terminal input reports presses but never releases, so this is how we
/// approximate "key is down": holding a key refreshes it via auto-repeat.
const HOLD_TICKS: i64 = 24; // 200 ms at 120 Hz

struct Options {
    interactive: bool,
    ticks: u32,
    render_every: u32,
    fast: bool,
    hash_only: bool,
    bench: Option<u64>,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            interactive: false,
            ticks: 1200, // 10 s
            render_every: 8,
            fast: false,
            hash_only: false,
            bench: None,
        }
    }
}

fn parse_args() -> Result<Options, String> {
    let mut opts = Options::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--interactive" => opts.interactive = true,
            "--fast" => opts.fast = true,
            "--hash-only" => opts.hash_only = true,
            "--bench" => {
                let v = args.next().ok_or("--bench needs a tick count")?;
                opts.bench = Some(v.parse().map_err(|_| "bad --bench")?);
            }
            "--ticks" => {
                let v = args.next().ok_or("--ticks needs a value")?;
                opts.ticks = v.parse().map_err(|_| "bad --ticks")?;
            }
            "--render-every" => {
                let v = args.next().ok_or("--render-every needs a value")?;
                opts.render_every = v.parse::<u32>().map_err(|_| "bad --render-every")?.max(1);
            }
            "--help" | "-h" => {
                println!(
                    "rl_sim\n\n  --interactive      drive with WASD (needs a TTY)\n  \
                     --ticks N          scripted run length in ticks (default 1200)\n  \
                     --render-every N   draw every N ticks in scripted mode (default 8)\n  \
                     --fast             don't pace the scripted mode to real time\n  \
                     --hash-only        print only the final state hash"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(opts)
}

/// The scripted demo input, purely a function of the tick number.
fn scripted_input(tick: u32) -> Input {
    match tick {
        0..=250 => Input::new(1.0, 0.0, false, false),
        251..=315 => Input::new(1.0, 0.0, true, false),
        316..=700 => Input::new(1.0, 0.75, false, false),
        701..=820 => Input::new(1.0, -1.0, false, true),
        _ => Input::new(1.0, 0.4, false, false),
    }
}

fn main() {
    let opts = match parse_args() {
        Ok(o) => o,
        Err(e) => {
            eprintln!("error: {e} (try --help)");
            std::process::exit(2);
        }
    };

    if let Some(ticks) = opts.bench {
        run_bench(ticks);
    } else if opts.interactive {
        run_interactive();
    } else {
        run_scripted(&opts);
    }
}

/// Headless throughput measurement. Not a micro-benchmark — it measures the
/// whole fixed step, which is what a server actually pays for.
fn run_bench(ticks: u64) {
    let mut world = World::new();
    let mut rng: u64 = 0x1234_5678_9ABC_DEF0;
    let mut next_input = || {
        rng = rng
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let a = ((rng >> 11) as f64) / ((1u64 << 53) as f64);
        Input::new(a, a * 2.0 - 1.0, a < 0.5, false)
    };

    // Warm up caches / branch predictors on a small slice first.
    for _ in 0..10_000 {
        let input = next_input();
        world.step(&input);
    }

    let start = Instant::now();
    for _ in 0..ticks {
        let input = next_input();
        world.step(&input);
    }
    let elapsed = start.elapsed().as_secs_f64();
    let per_sec = ticks as f64 / elapsed;
    println!(
        "{ticks} ticks in {elapsed:.4} s -> {:.1}M ticks/s ({:.0}x real time at {TICK_HZ} Hz)",
        per_sec / 1.0e6,
        per_sec / TICK_HZ as f64
    );
    println!("state hash after bench: {:#018x}", world.state_hash());
}

fn run_scripted(opts: &Options) {
    let mut world = World::new();
    let mut stdout = io::stdout();
    let frame_interval = Duration::from_secs_f64(opts.render_every as f64 * DT);
    let start = Instant::now();

    if !opts.hash_only {
        // Alternate screen buffer keeps the user's scrollback clean, and
        // clearing each frame sidesteps any terminal-size dependent drift.
        write!(stdout, "\x1b[?1049h\x1b[2J\x1b[H").ok();
    }

    for tick in 0..opts.ticks {
        world.step(&scripted_input(tick));

        if opts.hash_only || tick % opts.render_every != 0 {
            continue;
        }

        let rendered = tick / opts.render_every;
        write!(stdout, "\x1b[2J\x1b[H{}", render(&world, "scripted")).ok();
        stdout.flush().ok();

        if !opts.fast {
            // Pace to real time so the animation reads at the right speed.
            let due = frame_interval * (rendered + 1);
            let elapsed = start.elapsed();
            if due > elapsed {
                thread::sleep(due - elapsed);
            }
        }
    }

    if !opts.hash_only {
        write!(stdout, "\x1b[?1049l").ok();
        stdout.flush().ok();
    }

    let hash = world.state_hash();
    if opts.hash_only {
        println!("{hash:#018x}");
    } else {
        println!("\nfinal state hash: {hash:#018x}  (tick {})", world.tick);
    }
}

struct RawGuard(String);

impl Drop for RawGuard {
    fn drop(&mut self) {
        let _ = std::process::Command::new("stty").arg(&self.0).status();
        print!("\x1b[?1049l\x1b[?25h");
        let _ = io::stdout().flush();
    }
}

fn run_interactive() {
    if !std::io::stdin().is_terminal() {
        eprintln!("--interactive needs a terminal on stdin");
        std::process::exit(2);
    }

    let saved = std::process::Command::new("stty")
        .arg("-g")
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_default();
    if saved.is_empty() {
        eprintln!("could not read terminal settings (is `stty` available?)");
        std::process::exit(2);
    }
    let _guard = RawGuard(saved.clone());
    let _ = std::process::Command::new("stty")
        .arg("raw")
        .arg("-echo")
        .status();
    print!("\x1b[?1049h\x1b[2J\x1b[?25l");

    // Reader thread: raw mode delivers one byte per keypress.
    let (tx, rx) = mpsc::channel::<u8>();
    thread::spawn(move || {
        let mut stdin = io::stdin();
        let mut buf = [0u8; 1];
        while let Ok(1) = stdin.read(&mut buf) {
            if tx.send(buf[0]).is_err() {
                break;
            }
        }
    });

    let mut world = World::new();
    let mut stdout = io::stdout();
    let mut quit = false;
    let mut acc = Instant::now();
    let mut tick: i64 = 0;
    let mut last_seen = [-1_000_000i64; 256];

    while !quit {
        while let Ok(k) = rx.try_recv() {
            if k == 3 || k == b'q' {
                quit = true;
                continue;
            }
            last_seen[k.to_ascii_lowercase() as usize] = tick;
        }
        let input = Input::new(
            axis_pair(&last_seen, b'w', b's', tick),
            axis_pair(&last_seen, b'a', b'd', tick),
            held(&last_seen, b' ', tick),
            held(&last_seen, b'z', tick),
        );
        world.step(&input);
        tick += 1;

        write!(
            stdout,
            "\x1b[2J\x1b[H{}",
            render(&world, "WASD drive  space boost  z drift  q quit")
        )
        .ok();
        stdout.flush().ok();

        acc += Duration::from_secs_f64(DT);
        let now = Instant::now();
        if acc > now {
            thread::sleep(acc - now);
        } else {
            acc = now;
        }
    }
}

fn held(last_seen: &[i64; 256], key: u8, tick: i64) -> bool {
    tick - last_seen[key as usize] < HOLD_TICKS
}

fn axis_pair(last_seen: &[i64; 256], positive: u8, negative: u8, tick: i64) -> f64 {
    match (
        held(last_seen, positive, tick),
        held(last_seen, negative, tick),
    ) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}

fn render(world: &World, subtitle: &str) -> String {
    let mut grid = vec![vec![' '; COLS]; ROWS];

    for cell in grid[0].iter_mut() {
        *cell = '-';
    }
    for cell in grid[ROWS - 1].iter_mut() {
        *cell = '-';
    }
    for row in grid.iter_mut() {
        row[0] = '|';
        row[COLS - 1] = '|';
    }

    // At 2 m per cell the car is smaller than one cell, so draw a marker for
    // the body plus one for the nose rather than trying to rasterise it.
    let (bx, by) = cell_of(world.ball.pos.x, world.ball.pos.z);
    set_cell(&mut grid, bx, by, 'O');

    let (cx, cy) = cell_of(world.car.pos.x, world.car.pos.z);
    set_cell(&mut grid, cx, cy, '@');

    let nose = world.car.pos + world.car.heading * (CAR_HALF_LENGTH + 0.7);
    let (nx, ny) = cell_of(nose.x, nose.z);
    set_cell_if_free(&mut grid, nx, ny, '+');

    let mut lines: Vec<String> = Vec::new();
    lines.push(format!("rl_sim  {subtitle}"));
    lines.push(format!(
        "tick {:>6} {:>5.1}s   car {:>5.1} m/s  boost {:>5.1}   ball {:>5.1} m/s  height {:>5.2} m",
        world.tick,
        world.tick as f64 * DT,
        world.car.speed(),
        world.car.boost,
        world.ball.vel.length(),
        world.ball.pos.y,
    ));
    for row in &grid {
        lines.push(row.iter().collect());
    }
    lines.push("O ball   @ car   + nose              (grid: 2 m per cell)".to_string());

    // Pad every line to a fixed width, otherwise a line that got shorter
    // would leave residue from the previous frame on screen.
    const WIDTH: usize = 64;
    let mut out = String::new();
    for line in lines {
        let len = line.chars().count();
        out.push_str(&line);
        for _ in len..WIDTH {
            out.push(' ');
        }
        out.push('\n');
    }
    out
}

fn set_cell(grid: &mut [Vec<char>], col: i64, row: i64, glyph: char) {
    if col < 0 || row < 0 || col as usize >= COLS || row as usize >= ROWS {
        return;
    }
    grid[row as usize][col as usize] = glyph;
}

fn set_cell_if_free(grid: &mut [Vec<char>], col: i64, row: i64, glyph: char) {
    if col < 0 || row < 0 || col as usize >= COLS || row as usize >= ROWS {
        return;
    }
    let cell = &mut grid[row as usize][col as usize];
    if *cell == ' ' {
        *cell = glyph;
    }
}

fn cell_of(x: f64, z: f64) -> (i64, i64) {
    let col = ((x + 40.0) / 80.0 * COLS as f64).floor() as i64;
    let row = ((25.0 - z) / 50.0 * ROWS as f64).floor() as i64;
    (col.clamp(0, COLS as i64 - 1), row.clamp(0, ROWS as i64 - 1))
}
