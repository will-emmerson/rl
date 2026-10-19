#!/usr/bin/env sh
# Render an rl_view screenshot and keep only a downscaled JPEG for review.
#
#   tools/shot.sh screenshots/chase.jpg 250
#   tools/shot.sh screenshots/goal.jpg --cam 0,6,34 --look 0,0,48
#
# The PNG is temporary. Agent threads must never read it: the image is stored
# as base64 in the thread's tool history and re-sent on every later turn (see
# the screenshot trap in AGENTS.md). Only the JPEG is left behind.
set -eu

if [ "$#" -lt 1 ]; then
    echo "usage: $0 OUT.jpg [TICK | --cam X,Y,Z --look X,Y,Z]" >&2
    exit 2
fi

out=$1
shift
png="${out%.*}.png"

mkdir -p "$(dirname "$out")"
cargo run -p rl_view -- --screenshot "$png" "$@"
ffmpeg -y -loglevel error -i "$png" -vf scale=1280:-1 -q:v 4 "$out"
rm -f "$png"
ls -lh "$out"
