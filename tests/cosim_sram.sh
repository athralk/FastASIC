#!/usr/bin/env bash
# SRAM inference check: tests/cosim.sh on a design whose memories must have become IHP SRAM
# macros (simulated with the PDK's functional model), 4000 clock cycles.
# usage: tests/cosim_sram.sh <output_test> <design.v>      (COSIM_MUTATE is passed through)
set -euo pipefail
"$1" -q "$2" | grep -q "SRAM inference: .* -> " || { echo "no SRAM was inferred"; exit 2; }
exec bash "$(dirname "$0")/cosim.sh" "$1" "$2" 4000
