# Verilator build of sim/nb2_ddr_mux_tb.sv -> build/vl/obj_ddrmux/nb2_ddr_mux_tb.exe
set -e
mkdir -p "$(dirname "$0")/../../build/vl"; cd "$(dirname "$0")/../../build/vl"
R=../..
verilator --binary --timing -j 8 --top-module nb2_ddr_mux_tb -O3 -Wno-fatal -Wno-WIDTH -Wno-TIMESCALEMOD \
  -Wno-INITIALDLY -Wno-BLKANDNBLK -MAKEFLAGS "OPT_SLOW=-O1 OPT_GLOBAL=-O1 OPT_FAST=-O2" --Mdir obj_ddrmux \
  -o nb2_ddr_mux_tb $R/rtl/nb2/nb2_ddr_mux.sv $R/sim/models/ddram_model.sv $R/sim/nb2_ddr_mux_tb.sv 2>&1 \
  | grep -v "^%Warning\|^ *:\|^ *|\|^ *[0-9]* |" | tail -20
