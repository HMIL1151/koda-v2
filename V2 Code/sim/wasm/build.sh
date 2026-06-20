#!/usr/bin/env bash
# Compile the firmware control core + embind bindings to WebAssembly.
# Output (koda-core.mjs/.wasm) lands in ../web and is imported by both the browser sim and
# the node tests. The SRC list below is the single "core source list" — the exact files the
# PlatformIO firmware build also compiles, so the sim can't run stale logic.
#
#   build.sh            release build (-O2)
#   build.sh --debug    debug build (-g -O0): embeds DWARF so you can set breakpoints in
#                       the actual .cpp files in Chrome DevTools (see sim/README.md).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CORE="$HERE/../../esp32-master"
OUT="$HERE/../web"
mkdir -p "$OUT"

# Activate Emscripten (installed via emsdk).
source "$HOME/emsdk/emsdk_env.sh" >/dev/null 2>&1

OPT=(-O2)
if [[ "${1:-}" == "--debug" ]]; then
  OPT=(-g -O0)            # full DWARF, no optimisation → source-level WASM debugging
  echo "Debug build (source-level WASM debugging enabled)"
fi

SRC=(
  "$CORE/src/kinematics/inverse_kinematics.cpp"
  "$CORE/src/gait/gait.cpp"
  "$CORE/src/control/leg.cpp"
  "$CORE/src/control/robot.cpp"
  "$CORE/src/control/balance.cpp"
  "$CORE/src/control/ground_contact.cpp"
  "$CORE/src/control/slope_estimator.cpp"
  "$CORE/src/control/terrain_monitor.cpp"
  "$HERE/bindings.cpp"
)

emcc "${SRC[@]}" \
  -I"$CORE/include" -I"$CORE/src" \
  -std=gnu++17 "${OPT[@]}" -lembind \
  -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=KodaCore \
  -sENVIRONMENT=web,node -sALLOW_MEMORY_GROWTH=1 \
  -o "$OUT/koda-core.mjs"

echo "Built $OUT/koda-core.mjs (+ .wasm)"
