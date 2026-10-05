# Vendored RTL

## `sdram.sv` — MiSTer SDRAM controller (M2)

Lineage:

1. MiSTer-devel `GBA_MiSTer` `rtl/sdram.sv` at commit `93790a023395bbd90e5eaf4dfb2cb5910afd55f5`
   (Sorgelig 2015-2019, parts from hamsterworks; GPL-3.0-or-later). Upstream blob SHA-1
   `35222d43b641f36cbca1d77531c08a3018f76472`.
2. Copied from the owner's NA-1/NA-2 core (`Namco_NA1_NA2_MiSTer/rtl/vendor/sdram.sv`, commit
   `9e07577614281700461320232b26c442db41abdc`), which carries the **NA-1 M15D delta**: `ch1_be[1:0]` byte
   enables driven onto DQM during writes, and `init` clearing latched requests and in-flight read returns.
   That copy's blob SHA-1 (LF) is `53bf37947f1e1ec661bb68d5e97e511c569454e9`, matching NA-1's README. The
   M15D delta is documented in full in the NA-1 repository's `rtl/vendor/README.md`. [HW-CONFIRMED in NA-1 at 100 MHz]
3. **NB-1 M2 delta (this repository):** the refresh interval is a module parameter,

   ```
   module sdram
   #(parameter [13:0] CYCLES_PER_REFRESH = 14'd780)
   ...
   localparam cycles_per_refresh  = CYCLES_PER_REFRESH;
   ```

   The default keeps upstream behaviour. `NB1.sv` passes `755` = 64 ms × 96.768 MHz / 8192 − 1, because the
   controller's average refresh period is `(cycles_per_refresh − 1)` clocks: 754 × 10.334 ns = 7.79 µs ≤ 7.8125 µs.
   Upstream 780 at 96.768 MHz would give 8.05 µs. Nothing else changed. Resulting blob SHA-1 (LF):
   `70acade132f64fb9a16daf487c5c27d7ee877c5e`.

4. **NB-1 M23 delta (this repository):** a new output `ch1_taken`, a one-clock pulse the cycle after the
   controller takes a latched ch1 request (the cycle it issues ACTIVE). After that the controller no longer reads
   `ch1_addr`/`ch1_din`/`ch1_be`/`ch1_rnw`, so `nb1_memory` (PIPE = 1) may pulse the next request then and the
   next ACTIVE follows the previous burst with no front-end turnaround. A pulse in the very cycle a request is
   taken would be lost (the IDLE branch clears `ch1_rq` after the OR-latch), which is why the front end waits
   for `ch1_taken`. Nothing else changed; an unconnected `ch1_taken` gives the upstream behaviour.

The file keeps its original copyright and GPL-3.0-or-later notice.

### Simulation note
ModelSim ASE 10.5b rejects three idioms Quartus accepts (`command`/`chip` used before declaration, an initialised
variable inside `always`, and `inout reg`). `scripts/test-m2.ps1` generates a mechanically edited copy in
`build/m2-sim/sdram_sim.sv` for simulation; the committed file is not edited for the simulator.

## `m37702/` — Mitsubishi M37702 CPU core + M37702M2 on-chip peripherals (M8, the C75)

| File | Module | Upstream path | SHA-1 (as copied, LF) |
|---|---|---|---|
| `m37702/na1_m37702.sv` | `na1_m37702` (CPU core) | `rtl/na1/na1_m37702.sv` | `321b5f45449451c4f4b7de9476b47d951e520106` |
| `m37702/na1_m37702_decode.sv` | package `na1_m37702_pkg` (opcode decode/cycle table) | `rtl/na1/na1_m37702_decode.sv` | `cfbb1fbdb5b4cb6cb86f059a4a56cfa07cb8145e` |
| `m37702/na1_c69_sfr.sv` | `na1_c69_sfr` (SFRs $00-$7F: ports, timers, A-D, UART stubs, interrupt controller) | `rtl/na1/na1_c69_sfr.sv` | `9a7c6d7b2bf5c13a41efd95770df60753f983ea2` |

- **Source:** the owner's Namco NA-1/NA-2 MiSTer core, `https://github.com/kyledlester/Namco_NA1_NA2_MiSTer`,
  commit `6ccbae871e5780cb028bb8fed66f5258dc9db0cc` ("Close timing at clk_sys 100.226 MHz"), the latest change
  to these three files. There the core runs the genuine C69/C70 BIOS in a released, hardware-proven core.
- **License:** GPL-3.0 (that repository's `LICENSE`), the same as this project.
- **Behavioural contract (upstream headers):** MAME 0.289 `src/devices/cpu/m37710/` (opcode semantics, cycle
  counts padded per instruction, interrupt entry, `m37702m2_device` SFR map, timers, A-D, interrupt resolver).
  `na1_m37702_decode.sv` is generated from MAME's m37710 opcode tables by the NA-1 repository's
  `scripts/m20b-decode.py`.
- **Modifications: three (M19, M22, M24).** `na1_m37702_decode.sv` is byte-identical to upstream. **M24 (timing only,
  no behaviour change):** the `na1_c69_sfr.sv` interrupt resolver computes, per priority, whether a line is pending and
  the highest such line without `ipl`, then takes the highest priority above `ipl` and its line, so `ipl` enters last
  (same result as the scan; `sim/m24_irq_resolver_tb.sv`, 4.8 M cases). In `na1_m37702.sv` the PSH/PUL register
  mask is kept in visit order and shifted as `step` advances, so the current bit is always `mask8[0]` (same states
  and decisions; `sim/m24_c75_pshpul_tb.sv` runs it in lock-step with the M23 core over every mask and width mode). **M22 (timing only, no
  behaviour change):** `na1_m37702.sv` registers the 16-bit decimal ADC/SBC high byte at `S_OPER` for both low-byte
  carries (`dech`, `dbh0`/`dbh1`, the same idea as upstream's own M20C low-byte cut), and takes the MPY Z flag from
  the operands (a non-truncated product is zero exactly when a factor is). Exhaustive equivalence:
  `sim/m22_c75_alu_tb.sv`; C75/MAME lock-step: `scripts/test-m8.ps1`. **M19:**
  `na1_c69_sfr.sv` gains a `COUNT_STOP` parameter (default 0 = the upstream behaviour, unchanged): with 1, a
  count-start bit written 1 -> 0 stops that timer at the instruction boundary (M37702 datasheet; MAME never stops a
  started timer). The NB-1 sets it: Nebulas Ray's sound driver restarts timer A0 every 60 Hz frame and stops it
  after it fires, and without the stop the RTL took A0 twice per frame, doubling music and voice-cue tempo
  (`docs/M19_RESEARCH.md`). The SHA-1 above is the upstream copy; `git diff` against that commit shows the
  parameter. The module names keep their `na1_` prefix so they can be diffed against upstream directly. `na1_c69_sfr` is the generic M37702M2 peripheral block despite
  its name (the C69 and the C75 are both `m37702m2` in MAME `namcomcu.cpp`). All NB-1 board glue (address map,
  shared RAM, data ROM, ports, interrupt sources) is in `rtl/nb1/nb1_c75.sv`.
- **Configuration used:** `na1_c69_sfr #(.MAME_EARLY(1), .COUNT_STOP(1))`. The MAME timer-rounding artefact it
  reproduces also holds for the C75's 16.128 MHz input clock: 2·⌊10¹⁸/16,128,000⌋ = 124,007,936,506 as <
  ⌊10¹⁸/8,064,000⌋ = 124,007,936,507 as. The CPU runs one MAME cycle per two `ce_c75` ticks (8.064 MHz).
- **Alternative considered:** `SYSTEM11_MiSTer/rtl/m37702.vhd` (Namco C76). Its own header describes it as a
  "boot-path tier" core (8-bit bus; decimal mode, indirect modes, prefixes and interrupts as later phases), so
  it was not chosen. See `docs/M8_RESEARCH.md` section 2.

## `crt_adjust.sv` — MiSTer-CRT-Adjust, UNMODIFIED upstream copy (M17)

`crt_adjust.sv` is from the **MiSTer-CRT-Adjust** project by Umberto Parisi (rmonic79), with Andrea Bogazzi
(@asturur): the owner's local copy `MiSTer-CRT-Adjust-master`, file `rtl/crt_adjust.sv`. It is byte-identical to the
copy the owner's NA-1/NA-2 core vendors and hardware-confirmed there (M26). Licence: GNU GPL v3 or later (header
retained; compatible with this repository). All NB-1 integration lives in `rtl/nb1/nb1_crt_adjust.sv`. Not
vendored, as in NA-1/NA-2: `crt_adjust_sys.sv` (needs `sys/sys_top.v` edits) and `crt_vsize.sv` (V-Size retimes
the line rate and the HSync pulse).
