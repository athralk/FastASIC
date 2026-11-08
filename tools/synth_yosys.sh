#!/usr/bin/env bash
# Synthesizes RTL to an sg13cmos5l gate-level netlist with Yosys + ABC (the core of what
# OpenLane's synthesis step does), for the timing comparisons.
# usage: tools/synth_yosys.sh <design.v> <top> <out.v> [corner lib, default typ]
set -euo pipefail
LIB=${4:-${PDK_ROOT:-$HOME/ttsetup/pdk}/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_typ_1p20V_25C.lib}
yosys -q -p "read_verilog $1; synth -top $2 -flatten; dfflibmap -liberty $LIB; abc -liberty $LIB;
             opt_clean -purge; hilomap -hicell sg13cmos5l_tiehi L_HI -locell sg13cmos5l_tielo L_LO;
             setundef -zero; opt_clean; write_verilog -noattr -noexpr -nohex -nodec $3"
