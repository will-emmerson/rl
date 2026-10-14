#!/usr/bin/env python3
"""rlacd - a Linux/Proton port of RLArenaCollisionDumper.

Dumps Rocket League's arena collision meshes into RocketSim's `.cmf` format,
for Linux, where the original (a standalone Windows program that injects into
Rocket League's process) can't run.

This is a faithful port of the technique in ZealanL/RLArenaCollisionDumper:

  1. find the Rocket League process (running under Proton/Wine),
  2. locate its `RocketLeague.exe` PE image in /proc/<pid>/maps,
  3. pattern-scan the mapped code for a `btDynamicsWorld` member function,
  4. ptrace-breakpoint that function to capture RCX, the `btDynamicsWorld*`
     (the Windows tool instead injects shellcode that stores RCX; on Linux a
     breakpoint plus PTRACE_GETREGS is simpler and touches no code),
  5. walk the Bullet collision objects and write each triangle mesh out.

Memory is read with `process_vm_readv`; the breakpoint uses `ptrace`. Both need
the same privilege the Windows tool needs (Administrator there, root here),
because Yama's `ptrace_scope` blocks attaching to another user's process by
default. So run it as root, or set `ptrace_scope=0`.

Be in Free Play when you run it: the meshes are read live from the running game.

Usage:
    sudo python3 tools/rlacd.py                    # -> ./collision_meshes/<mode>/
    sudo python3 tools/rlacd.py --out /tmp/meshes
    sudo python3 tools/rlacd.py --list-maps        # what did it find?
    sudo python3 tools/rlacd.py --pid 12345 --mode soccar

The output layout matches what RocketSim v3 scans exactly:
    <out>/soccar/mesh_0.cmf ... mesh_15.cmf
so the result can be dropped straight into `RL/collision_meshes/`.
"""

import argparse
import ctypes
import ctypes.util
import os
import signal
import struct
import sys
import time

# --------------------------------------------------------------------------
# x86-64 Linux constants
# --------------------------------------------------------------------------

PTRACE_PEEKTEXT = 1
PTRACE_POKETEXT = 4
PTRACE_CONT = 7
PTRACE_GETREGS = 12
PTRACE_SETREGS = 13
PTRACE_ATTACH = 16
PTRACE_DETACH = 17

SIGTRAP = 5

# `struct user_regs_struct` on x86-64 is 27 u64s; RCX is the 12th, RIP the 17th.
REGS_BYTES = 27 * 8
RCX = 11
RIP = 16

# The four code signatures from the original dumper's `Main.cpp`. Wildcards are
# `?`. These locate a btDynamicsWorld member function that runs every tick.
BTWORLD_PATTERNS = [
    "48 89 5C 24 ? 48 89 74 24 ? 57 48 83 EC 70 48 8B 05 ? ? ? ? 48 33 C4 48 89 44 24 ? 48 8B F1",
    "40 53 48 83 EC 30 48 8B 01 48 8B D9 0F 29 74 24 ? 0F 28 F1 FF 90 ? ? ? ? 48 8B 03 0F 28 CE 48 8B CB FF 90",
    "48 89 5C 24 ? 48 89 6C 24 ? 48 89 74 24 ? 57 41 56 41 57 48 83 EC 20 48 63 99",
    "40 53 57 41 57 48 81 EC ? ? ? ? 0F 29 B4 24",
]

# Bullet shape type for a triangle mesh (btTriangleMeshShape).
TRIANGLE_MESH_SHAPE_PROXYTYPE = 21

# Mesh count -> game mode, as in the original (`Reader.cpp`).
MODE_BY_MESH_COUNT = {16: "soccar", 12: "hoops", 2: "dropshot"}

# --------------------------------------------------------------------------
# libc
# --------------------------------------------------------------------------


class IOVec(ctypes.Structure):
    _fields_ = [("iov_base", ctypes.c_void_p), ("iov_len", ctypes.c_size_t)]


libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)
libc.ptrace.restype = ctypes.c_long
libc.ptrace.argtypes = [ctypes.c_ulong, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
libc.process_vm_readv.restype = ctypes.c_ssize_t
libc.process_vm_readv.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(IOVec),
    ctypes.c_ulong,
    ctypes.POINTER(IOVec),
    ctypes.c_ulong,
    ctypes.c_ulong,
]


def log(msg):
    print(msg, flush=True)


def die(msg):
    print(f"error: {msg}", file=sys.stderr, flush=True)
    sys.exit(1)


# --------------------------------------------------------------------------
# /proc: find the game, its module mappings, its threads
# --------------------------------------------------------------------------


def find_rl_pid(module):
    """First PID whose command line mentions `module` (case-insensitive)."""
    needle = module.lower()
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmdline = f.read().replace(b"\x00", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if needle in cmdline.lower():
            return pid
    return None


def module_maps(pid, module):
    """(start, end, perms, path) for every mapping of `module`."""
    needle = module.lower()
    regions = []
    try:
        with open(f"/proc/{pid}/maps") as f:
            for line in f:
                parts = line.split()
                if len(parts) < 6:
                    continue
                path = parts[5]
                if needle not in path.lower():
                    continue
                start_s, end_s = parts[0].split("-")
                regions.append((int(start_s, 16), int(end_s, 16), parts[1], path))
    except OSError as e:
        die(f"cannot read /proc/{pid}/maps: {e}")
    return regions


def thread_ids(pid):
    try:
        return sorted(int(t) for t in os.listdir(f"/proc/{pid}/task"))
    except OSError as e:
        die(f"cannot list /proc/{pid}/task: {e}")


def all_maps(pid):
    """Every mapping of `pid`: (start, end, perms, path)."""
    regions = []
    try:
        with open(f"/proc/{pid}/maps") as f:
            for line in f:
                parts = line.split()
                if len(parts) < 5:
                    continue
                start_s, end_s = parts[0].split("-")
                path = parts[5] if len(parts) >= 6 else ""
                regions.append((int(start_s, 16), int(end_s, 16), parts[1], path))
    except OSError as e:
        die(f"cannot read /proc/{pid}/maps: {e}")
    return regions


def candidate_regions(pid, module):
    """Mappings to pattern-scan.

    The PE's code section is not always labelled with the exe's path (Proton
    maps it differently), so rather than trusting the pathname we take the
    module's address span from the mappings that *are* named, and scan every
    executable mapping inside that span. Falls back to all executable mappings
    of the process if that turns up nothing.
    """
    named = module_maps(pid, module)
    every = all_maps(pid)
    if named:
        base = min(r[0] for r in named)
        end = max(r[1] for r in named)
        execs = [r for r in every if "x" in r[2] and r[0] >= base and r[1] <= end]
        if execs:
            return execs
        span = [r for r in every if r[0] >= base and r[1] <= end]
        if span:
            return span
    return [r for r in every if "x" in r[2]]


# --------------------------------------------------------------------------
# memory reads (process_vm_readv)
# --------------------------------------------------------------------------

_CHUNK = 1 << 20


def vm_read(pid, addr, size):
    """Read `size` bytes at `addr` from `pid`. Needs ptrace-attach privilege."""
    out = bytearray()
    off = 0
    while off < size:
        n = min(_CHUNK, size - off)
        buf = ctypes.create_string_buffer(n)
        local = IOVec(ctypes.cast(buf, ctypes.c_void_p), n)
        remote = IOVec(ctypes.c_void_p(addr + off), n)
        ctypes.set_errno(0)
        got = libc.process_vm_readv(
            pid, ctypes.byref(local), 1, ctypes.byref(remote), 1, 0
        )
        if got < 0:
            err = ctypes.get_errno()
            if err == 1:  # EPERM
                die(
                    "process_vm_readv was denied. Run with sudo (or set "
                    "kernel.yama.ptrace_scope=0)."
                )
            raise OSError(err, os.strerror(err))
        if got == 0:
            break
        out.extend(buf.raw[:got])
        off += got
        if got < n:
            break
    return bytes(out)


def u32(pid, addr):
    return struct.unpack("<I", vm_read(pid, addr, 4))[0]


def u64(pid, addr):
    return struct.unpack("<Q", vm_read(pid, addr, 8))[0]


# --------------------------------------------------------------------------
# pattern scanning
# --------------------------------------------------------------------------


def parse_pattern(text):
    out = []
    for tok in text.split():
        out.append(None if tok == "?" else int(tok, 16))
    return out


def matches(data, i, pat):
    for j, v in enumerate(pat):
        if v is not None and data[i + j] != v:
            return False
    return True


def scan_one(data, base, pat):
    n = len(pat)
    if n == 0:
        return None
    # Fast path: jump on the leading run of fixed bytes with bytes.find.
    prefix = bytearray()
    for v in pat:
        if v is None:
            break
        prefix.append(v)
    if len(prefix) >= 4:
        needle = bytes(prefix)
        start = 0
        while True:
            i = data.find(needle, start)
            if i < 0:
                return None
            if matches(data, i, pat):
                return base + i
            start = i + 1
    first = next((j for j, v in enumerate(pat) if v is not None), None)
    if first is None:
        return None
    limit = len(data) - n
    for i in range(limit + 1):
        if data[i + first] == pat[first] and matches(data, i, pat):
            return base + i
    return None


def find_btworld_function(pid, regions):
    """Scan the module's mappings for the first matching signature."""
    ordered = sorted(regions, key=lambda r: ("x" not in r[2], r[0]))
    for start, end, perms, path in ordered:
        size = end - start
        if size <= 0:
            continue
        log(f"scanning {size / 1e6:.1f} MB at {start:#x} ({perms}) {path}")
        try:
            data = vm_read(pid, start, size)
        except OSError as e:
            log(f"  (skipped: {e})")
            continue
        for p in BTWORLD_PATTERNS:
            addr = scan_one(data, start, parse_pattern(p))
            if addr is not None:
                log(f"found signature at {addr:#x}")
                return addr
    return None


# --------------------------------------------------------------------------
# ptrace: capture RCX at a breakpoint
# --------------------------------------------------------------------------


def _ptrace(req, pid, addr=0, data=0):
    ctypes.set_errno(0)
    res = libc.ptrace(req, pid, ctypes.c_void_p(addr), ctypes.c_void_p(data))
    if res == -1:
        err = ctypes.get_errno()
        if err != 0:
            raise OSError(err, os.strerror(err))
    return res


def peek_word(pid, addr):
    ctypes.set_errno(0)
    res = libc.ptrace(PTRACE_PEEKTEXT, pid, ctypes.c_void_p(addr), None)
    err = ctypes.get_errno()
    if res == -1 and err != 0:
        raise OSError(err, os.strerror(err))
    return res & 0xFFFFFFFFFFFFFFFF


def get_regs(pid):
    buf = ctypes.create_string_buffer(REGS_BYTES)
    _ptrace(PTRACE_GETREGS, pid, 0, ctypes.addressof(buf))
    return struct.unpack("<27Q", buf.raw)


def set_regs(pid, vals):
    buf = ctypes.create_string_buffer(REGS_BYTES)
    struct.pack_into("<27Q", buf, 0, *vals)
    _ptrace(PTRACE_SETREGS, pid, 0, ctypes.addressof(buf))


def try_poke(pid, addr, word):
    """POKETEXT, returning False if the thread is not stopped."""
    try:
        _ptrace(PTRACE_POKETEXT, pid, addr, word)
        return True
    except OSError:
        return False


def force_restore(attached, target, original):
    """Put the original byte back, stopping a thread first if need be."""
    for tid in list(attached):
        if try_poke(tid, target, original):
            return
    for tid in list(attached):
        try:
            os.kill(tid, signal.SIGSTOP)
            os.waitpid(tid, 0)
        except OSError:
            continue
        if try_poke(tid, target, original):
            return


def attach(pid):
    """Attach to every thread of `pid`; returns the list of attached TIDs."""
    attached = []
    for tid in thread_ids(pid):
        try:
            _ptrace(PTRACE_ATTACH, tid)
            os.waitpid(tid, 0)
            attached.append(tid)
        except OSError as e:
            log(f"  (could not attach to thread {tid}: {e})")
    return attached


def intercept_rcx(pid, attached, target, timeout=15.0):
    """Breakpoint `target`, run, and return RCX from the first hit."""
    leader = attached[0]
    original = peek_word(leader, target)
    patched = (original & ~0xFF) | 0xCC  # INT3 over the first byte
    _ptrace(PTRACE_POKETEXT, leader, target, patched)

    for tid in attached:
        _ptrace(PTRACE_CONT, tid, 0, 0)

    log("waiting for the function to run (be in Free Play, not paused)...")
    deadline = time.monotonic() + timeout
    rcx = None
    restored = False
    try:
        while rcx is None:
            if time.monotonic() > deadline:
                die(
                    "timed out waiting for the breakpoint. Make sure you are in "
                    "Free Play and the game is not paused."
                )
            try:
                tid, status = os.waitpid(-1, 0)
            except ChildProcessError:
                die("all game threads exited while waiting.")

            if not os.WIFSTOPPED(status):
                if tid in attached:
                    attached.remove(tid)
                if not attached:
                    die("all game threads exited while waiting.")
                continue

            sig = os.WSTOPSIG(status)
            regs = get_regs(tid)
            if sig == SIGTRAP and regs[RIP] - 1 == target:
                rcx = regs[RCX]
                hijacked = [tid]
                # Any other thread already parked on the same INT3 needs the
                # same rewind below, or it would resume mid-instruction.
                while True:
                    try:
                        t2, s2 = os.waitpid(-1, os.WNOHANG)
                    except ChildProcessError:
                        break
                    if t2 == 0:
                        break
                    if not os.WIFSTOPPED(s2):
                        continue
                    r2 = get_regs(t2)
                    if os.WSTOPSIG(s2) == SIGTRAP and r2[RIP] - 1 == target:
                        hijacked.append(t2)
                    else:
                        _ptrace(PTRACE_CONT, t2, 0, os.WSTOPSIG(s2))
                # Restore the real instruction, then rewind each hijacked thread
                # to re-execute it from the start. `hijacked[0]` is stopped, so
                # this poke is guaranteed to land.
                restored = try_poke(hijacked[0], target, original)
                for t in hijacked:
                    r = list(get_regs(t))
                    r[RIP] = target
                    set_regs(t, r)
                    _ptrace(PTRACE_CONT, t, 0, 0)
                break
            # Not ours: pass the signal on and keep waiting.
            _ptrace(PTRACE_CONT, tid, 0, sig if sig not in (SIGTRAP, 19) else 0)
    finally:
        # On any early exit (timeout, error), make sure the INT3 is gone.
        if not restored:
            force_restore(attached, target, original)
    return rcx


# --------------------------------------------------------------------------
# Bullet walk (struct offsets from the original's BulletStructs.h)
# --------------------------------------------------------------------------


def read_meshes(pid, bt_world):
    # btDynamicsWorld { void* vtable; btAlignedObjectArray collisionObjects; }
    coll = bt_world + 8
    count = u32(pid, coll + 4)  # .size
    data_ptr = u64(pid, coll + 0x10)  # .data
    log(f"collision objects: {count}")

    meshes = []
    skipped = 0
    for i in range(count):
        obj = u64(pid, data_ptr + i * 8)
        if obj == 0:
            continue
        shape = u64(pid, obj + 0xD0)  # btCollisionObject.collisionShape
        if shape == 0:
            continue
        if u32(pid, shape + 8) != TRIANGLE_MESH_SHAPE_PROXYTYPE:
            continue

        smi = u64(pid, shape + 0x40)  # btTriangleMeshShape.stridingMeshInterface
        if smi == 0:
            skipped += 1
            continue

        # btTriangleIndexVertexArray.indexedMeshes: `.data` (at array+0x10)
        # points straight at the first btIndexedMesh, which the array stores by
        # value. So the pointer value *is* the struct's address (no extra deref).
        im = u64(pid, smi + 0x20 + 0x10)
        num_tris = u32(pid, im + 0)
        tri_ptr = u64(pid, im + 8)
        num_verts = u32(pid, im + 20)
        vert_ptr = u64(pid, im + 24)

        # Sanity-check the counts before trusting them to size reads (also
        # drops the tiny dropshot tile / plane meshes, as the original does).
        if not (5 < num_verts <= 5_000_000 and 0 < num_tris <= 5_000_000):
            skipped += 1
            continue

        # btTransform: basis @ +0x10 (3x btVector3, 16 bytes each), origin @ +0x40
        raw_basis = vm_read(pid, obj + 0x10, 48)
        b = struct.unpack("<12f", raw_basis)
        basis = [(b[0], b[1], b[2]), (b[4], b[5], b[6]), (b[8], b[9], b[10])]
        rotated = basis != [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        ox, oy, oz = struct.unpack("<3f", vm_read(pid, obj + 0x40, 12))

        tris_raw = vm_read(pid, tri_ptr, num_tris * 12)
        verts_raw = vm_read(pid, vert_ptr, num_verts * 12)

        indices = struct.unpack(f"<{num_tris * 3}i", tris_raw)

        verts = []
        for x, y, z in struct.iter_unpack("<3f", verts_raw):
            if rotated:
                # basis.Rotate(v): result_k = basis[0][k]*x + basis[1][k]*y + basis[2][k]*z
                x, y, z = (
                    basis[0][0] * x + basis[1][0] * y + basis[2][0] * z,
                    basis[0][1] * x + basis[1][1] * y + basis[2][1] * z,
                    basis[0][2] * x + basis[1][2] * y + basis[2][2] * z,
                )
            verts.append((x + ox, y + oy, z + oz))

        meshes.append((indices, verts))
        log(
            f"  mesh {len(meshes) - 1}: {num_tris} tris, {num_verts} verts"
            f"{' (rotated)' if rotated else ''}"
        )

    log(f"meshes found: {len(meshes)} (skipped {skipped})")
    return meshes


def write_cmf(path, indices, verts):
    flat = [c for v in verts for c in v]
    with open(path, "wb") as f:
        f.write(struct.pack("<ii", len(indices) // 3, len(verts)))
        f.write(struct.pack(f"<{len(indices)}i", *indices))
        f.write(struct.pack(f"<{len(flat)}f", *flat))


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def capture(pid, module, path):
    """Dump /proc maps plus the module's executable code to `path`."""
    with open(f"/proc/{pid}/maps") as f:
        with open(path + ".maps", "w") as out:
            out.write(f.read())

    named = module_maps(pid, module)
    every = all_maps(pid)
    base = min((r[0] for r in named), default=None)
    end = max((r[1] for r in named), default=None)

    chosen = []
    for r in every:
        if "x" not in r[2]:
            continue
        in_span = base is not None and r[0] >= base and r[1] <= end
        if not r[3] or "rocketleague" in r[3].lower() or in_span:
            chosen.append(r)
    if not chosen:
        die("no executable mappings found to capture.")

    log(f"capturing {len(chosen)} executable region(s):")
    total = 0
    matches = []
    with open(path, "wb") as f:
        f.write(b"RLACDCAP1\n")
        for start, end_, _perms, p in chosen:
            size = end_ - start
            data = vm_read(pid, start, size)
            if len(data) < size:
                log(f"  warning: short read at {start:#x} ({len(data)}/{size})")
                data += b"\x00" * (size - len(data))
            f.write(struct.pack("<QQ", start, size))
            f.write(data)
            total += size
            log(f"  {size / 1e6:.1f} MB at {start:#x} {p or '(anon)'}")
            for i, pat in enumerate(BTWORLD_PATTERNS):
                a = scan_one(data, start, parse_pattern(pat))
                if a is not None:
                    log(f"    -> signature {i} matched at {a:#x}")
                    matches.append((i, a))

    log(f"wrote {path} ({total / 1e6:.1f} MB) and {path}.maps")
    if matches:
        log("signature matches: " + ", ".join(f"#{i}@{a:#x}" for i, a in matches))
    else:
        log("no btDynamicsWorld signature matched the captured code.")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--pid", type=int, help="Rocket League PID (else auto-detected)")
    ap.add_argument(
        "--module",
        default="RocketLeague.exe",
        help="module name to look for (default: RocketLeague.exe)",
    )
    ap.add_argument("--out", default="./collision_meshes", help="output directory")
    ap.add_argument(
        "--mode",
        choices=["soccar", "hoops", "dropshot", "custom"],
        help="override the game-mode folder",
    )
    ap.add_argument(
        "--list-maps", action="store_true", help="show the module's mappings and exit"
    )
    ap.add_argument(
        "--capture",
        metavar="FILE",
        help="dump /proc maps + executable code to FILE and exit (needs no ptrace)",
    )
    ap.add_argument(
        "--address",
        type=lambda s: int(s, 0),
        help="use this absolute function address instead of pattern scanning",
    )
    ap.add_argument(
        "--timeout", type=float, default=15.0, help="seconds to wait for the breakpoint"
    )
    args = ap.parse_args()

    if os.geteuid() != 0:
        log("note: not running as root; ptrace usually needs sudo here.")

    pid = args.pid
    if pid is None:
        pid = find_rl_pid(args.module)
        if pid is None:
            die(
                f"could not find a process running {args.module}. Start Rocket "
                "League and load into Free Play first."
            )
    log(f"Rocket League PID: {pid}")

    regions = module_maps(pid, args.module)
    if args.list_maps:
        log(f"all mappings of PID {pid}:")
        for start, end, perms, path in all_maps(pid):
            mark = "X" if "x" in perms else " "
            log(f"  {mark} {start:#012x}-{end:#012x} {perms} {path or '(anon)'}")
        return
    if args.capture:
        capture(pid, args.module, args.capture)
        return
    if not regions:
        die(
            f"no {args.module} mappings in /proc/{pid}/maps. Try --list-maps "
            "and/or a different --module name."
        )

    candidates = [] if args.address else candidate_regions(pid, args.module)
    if not args.address and not candidates:
        die("no scannable mappings found; try --list-maps to see the layout.")

    attached = attach(pid)
    if not attached:
        die("could not attach to any game thread (need sudo / ptrace_scope=0).")
    log(f"attached to {len(attached)} thread(s)")

    try:
        if args.address:
            target = args.address
            log(f"using address {target:#x} (--address)")
        else:
            log(f"scanning {len(candidates)} candidate region(s)")
            target = find_btworld_function(pid, candidates)
        if target is None:
            die(
                "none of the btDynamicsWorld signatures matched. The game may "
                "have been updated, or the module was not the one expected."
            )

        bt_world = intercept_rcx(pid, attached, target, args.timeout)
        if not bt_world:
            die("captured a null btDynamicsWorld pointer.")
        log(f"btDynamicsWorld at {bt_world:#x}")

        meshes = read_meshes(pid, bt_world)
    finally:
        for tid in attached:
            try:
                _ptrace(PTRACE_DETACH, tid, 0, 0)
            except OSError:
                pass  # running threads are detached by the kernel on exit

    if not meshes:
        die("no collision meshes found. Make sure you are in Free Play.")

    mode = args.mode
    if mode is None:
        mode = MODE_BY_MESH_COUNT.get(len(meshes))
        if mode is None:
            die(
                f"unexpected mesh count {len(meshes)} (expected "
                f"{sorted(MODE_BY_MESH_COUNT)}). Pass --mode to override."
            )
    log(f"detected game mode: {mode}")

    out_dir = os.path.join(args.out, mode)
    os.makedirs(out_dir, exist_ok=True)
    for i, (indices, verts) in enumerate(meshes):
        path = os.path.join(out_dir, f"mesh_{i}.cmf")
        write_cmf(path, indices, verts)
        log(f"saved {path}")
    log("done.")


if __name__ == "__main__":
    main()
