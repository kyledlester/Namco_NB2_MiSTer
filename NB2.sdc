# Namco NB-2 MiSTer core -- core timing constraints (sys/sys_top.sdc covers the framework).
# From the NB-1 core's NB1.sdc (docs/PROVENANCE.md): same clock, same SDRAM pin timing (rtl/nb2/nb2_sdram.sv keeps
# the vendored controller's physical interface), same TG68K / predecode multicycles (nb1_cpu unchanged; the
# nb2_main_bus predecode registers are kcap_*). The milestone notes below are the NB-1 history.
# TG68K kernel (M3, below), whose enable spacing is guaranteed by nb1_cpu.
derive_pll_clocks

# ---------------------------------------------------------------------------
# M2: SDRAM pins (rtl/vendor/sdram.sv, clocked by clk_sys; no second clock).
# Ported unchanged in value from the hardware-proven NA-1 core (NA1.sdc, M15C.1/M16),
# which runs the same controller at 100 MHz; only the clock pattern differs.
#
# The controller launches address/command/write data on clk_sys's rising edge.
# altddio_out drives SDRAM_CLK low on that edge and high on the falling edge, so
# the SDRAM sees an inverted copy of clk_sys.
set nb2_core_clock_pin [get_pins -compatibility_mode {*|pll|pll_inst|altera_pll_i|*|divclk}]
create_generated_clock -name SDRAM_CLK -source $nb2_core_clock_pin \
    -divide_by 1 -invert [get_ports {SDRAM_CLK}]

# MiSTer MemTest SDRAM-module limits (as used by NA-1): module clock-to-data /
# read-hold, and address/control/write-data setup/hold at the SDRAM pins.
set_input_delay -max -clock SDRAM_CLK 6.4 [get_ports {SDRAM_DQ[*]}]
set_input_delay -min -clock SDRAM_CLK 3.7 [get_ports {SDRAM_DQ[*]}]

# Read capture: the controller consumes the DQ sample taken on the second clk_sys
# rising edge after the launching SDRAM_CLK edge (data_ready_delay indexing, CL=2).
# A setup multicycle of 2 models that capture edge (NA-1 M16 analysis). No hold
# exception: TimeQuest then checks the next burst word against this edge, which
# is the real hold hazard.
set_multicycle_path -setup 2 -from [get_clocks {SDRAM_CLK}] \
    -to [get_clocks {*|pll|pll_inst|altera_pll_i|*|divclk}]

set nb2_sdram_outputs [get_ports {
    SDRAM_A[*] SDRAM_BA[*]
    SDRAM_nCS SDRAM_nWE SDRAM_nRAS SDRAM_nCAS
    SDRAM_DQMH SDRAM_DQML SDRAM_DQ[*]
}]
set_output_delay -max -clock SDRAM_CLK 1.6 $nb2_sdram_outputs
set_output_delay -min -clock SDRAM_CLK -0.9 $nb2_sdram_outputs

# ---------------------------------------------------------------------------
# M3: TG68K 68020 kernel (rtl/vendor/tg68k) inside nb1_cpu (rtl/nb1/nb1_cpu.sv).
# Contract, enforced by nb1_cpu's scheduler and checked by sim/m3_cpu_tb.sv (S1):
#   * every kernel register is enabled by clkena_in, and nb1_cpu never asserts
#     clkena_in on two edges closer than MIN_GAP = 3 clk_sys cycles, so kernel ->
#     kernel paths have 3 cycles (TG68K's own paths are ~21 ns on Cyclone V);
#   * the combinational kernel outputs (addr_out, data_write, busstate, nUDS/nLDS,
#     CACR/VBR) are sampled only by nb1_cpu's kcap_* registers, exactly
#     SAMPLE_DELAY = 2 cycles after an enable edge, and never used before that.
# Paths INTO the kernel (data_in, clkena_in; IPL is the M9 exception below) come from registers in nb1_cpu /
# nb1_main_bus through short logic and stay single-cycle. If MIN_GAP or
# SAMPLE_DELAY change in NB1.sv, change these numbers with them.
# Check the TimeQuest report: none of these filters may be reported as "ignored".
set nb2_tg68k [get_keepers -nocase {*|nb1_cpu:cpu|tg68kdotc_kernel:cpu|*}]
set nb2_kcap  [get_keepers -nocase {*|nb1_cpu:cpu|kcap_*}]
set_multicycle_path -setup 3 -from $nb2_tg68k -to $nb2_tg68k
set_multicycle_path -hold  2 -from $nb2_tg68k -to $nb2_tg68k
set_multicycle_path -setup 2 -from $nb2_tg68k -to $nb2_kcap
set_multicycle_path -hold  1 -from $nb2_tg68k -to $nb2_kcap
# M9: the interrupt level enters the kernel only from nb1_cpu's kin_ipl, which
# is loaded only on an enable edge (like a kernel register), so kin_ipl ->
# kernel has the same MIN_GAP = 3 cycles as kernel -> kernel.
set nb2_kin   [get_keepers -nocase {*|nb1_cpu:cpu|kin_*}]
set_multicycle_path -setup 3 -from $nb2_kin -to $nb2_tg68k
set_multicycle_path -hold  2 -from $nb2_kin -to $nb2_tg68k

derive_clock_uncertainty

# ---------------------------------------------------------------------------
# M22 (R-TIMING-1): nb1_main_bus PREDECODE registers (kcap_cls, kcap_wram, kcap_hit) load on nb1_cpu's
# pd_sample, which is exactly the kcap_* load (age == SAMPLE_DELAY after an enable edge; nb1_cpu.sv),
# from the same combinational kernel address (pd_addr = addr_out). The kernel registers change only on
# enable edges, so kernel -> these registers has the same SAMPLE_DELAY = 2 cycles as kernel -> kcap_*
# above. Proof and bench evidence: docs/M22_TIMING_CLOSURE.md.
set nb2_pd [get_keepers -nocase {*|nb2_main_bus:main_bus|kcap_*}]
set_multicycle_path -setup 2 -from $nb2_tg68k -to $nb2_pd
set_multicycle_path -hold  1 -from $nb2_tg68k -to $nb2_pd

# ---------------------------------------------------------------------------
# M22: framework HQ2x blender (sys/hq2x.sv module Blend, instantiated by sys/scandoubler.v inside
# arcade_video). Every Blend register (rule/df_rule, op0/i10/i20/i30, op/i1-i3, Result) is enabled only by
# its clk_en = Hq2x ce_in = scandoubler ce_x4i. With the NB-1 raster (ce_pix = clk_sys/16; the M17 CRT
# Adjust NCO is gated off whenever a scandoubler Fx or the forced scandoubler is active) ce_x4i fires at
# pixel clocks 4, 8, 12, 16: sim/m22_hq2x_ce_tb.sv measures 811,000 pulses over two frames, minimum
# spacing 4 clk_sys, never on consecutive clocks. So Blend -> Blend paths always have >= 2 cycles.
# Only exception: the first ~32 clk_sys after FPGA configuration, before the scandoubler has measured the
# pixel length (pixsz = 0), when ce_x4i may fire every clock; Blend holds no feedback state, so the worst
# case is a few wrong output pixels before any picture exists. Details: docs/M22_TIMING_CLOSURE.md.
set nb2_blend [get_keepers -nocase {*|Hq2x:Hq2x|Blend:blender|*}]
set_multicycle_path -setup 2 -from $nb2_blend -to $nb2_blend
set_multicycle_path -hold  1 -from $nb2_blend -to $nb2_blend
