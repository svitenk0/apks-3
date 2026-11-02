#!/usr/bin/env bash
# =============================================================================
# Функціональна симуляція I2C Master (Icarus Verilog)
#   режими: Standard-mode 100 кГц і Fast-mode 400 кГц (clk = 50 МГц)
#   результат: build/sim/tb_<f>.log, build/sim/tb_<f>.vcd (осцилограми для GTKWave)
# Використання: bash sim/run_sim.sh [100000 400000]
# =============================================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if ! command -v iverilog >/dev/null 2>&1 && [ -x /c/iverilog/bin/iverilog ]; then
    export PATH="/c/iverilog/bin:$PATH"                  # стандартний шлях інсталятора Windows
fi
OUT="$ROOT/build/sim"
mkdir -p "$OUT"
FREQS="${*:-100000 400000}"
RC=0

for F in $FREQS; do
    echo "==> SCL = $F Hz"
    iverilog -g2005 -Wall -Wno-timescale -P "tb_i2c_master.SCL_FREQ=$F" \
        -o "$OUT/tb_$F.vvp" rtl/*.v tb/*.v
    vvp -n "$OUT/tb_$F.vvp" "+vcd=$OUT/tb_$F.vcd" | tee "$OUT/tb_$F.log" | grep -v '^\[MON\]'
    grep -q "TEST PASSED" "$OUT/tb_$F.log" || RC=1
done

if [ "$RC" -eq 0 ]; then echo "SIM: SUCCESS"; else echo "SIM: FAILED" >&2; fi
exit "$RC"
