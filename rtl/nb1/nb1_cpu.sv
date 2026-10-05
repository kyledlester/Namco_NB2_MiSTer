// Namco NB-1 MiSTer core -- 68EC020 CPU wrapper (M3).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// TG68K.C (rtl/vendor/tg68k, 68020 mode) plus:
//   * the bus-cycle capture that turns TG68K's clock-enable bus into one
//     request per bus cycle for nb1_main_bus (bc_* below), and
//   * the SR-2 scheduler that decides when TG68K may take its next step.
// Design notes and evidence: docs/M3_RESEARCH.md sections 1-2, 5;
// docs/M3_IMPLEMENTATION.md sections 3-4.
//
// ---------------------------------------------------------------------------
// TG68K bus model [REFERENCE-CORE: TG68KdotC_Kernel.vhd, Minimig-AGA_MiSTer
// cpu_wrapper.v]: every kernel register is enabled by clkena_in. Between two
// enables the kernel presents ONE bus cycle on busstate/addr_out/nUDS/nLDS/
// nWr/data_write (busstate 00 = opcode fetch, 10 = data read, 11 = data
// write, 01 = no access). The next enable completes it: a read samples
// data_in on that edge. So "one enable = one CPU step", and a bus cycle is
// performed exactly once if it is serviced once between two enables.
//
// Timing contract (must match NB1.sdc; parameters exist to document it, not
// to be changed casually):
//   * SAMPLE_DELAY: kernel outputs are combinational (addr_out is an adder
//     over the register file) and are sampled into kcap_* registers exactly
//     SAMPLE_DELAY clk_sys cycles after an enable edge (multicycle
//     kernel -> kcap_* = SAMPLE_DELAY).
//   * MIN_GAP: two enables are always at least MIN_GAP cycles apart
//     (multicycle kernel -> kernel = MIN_GAP). TG68K's own paths are ~21 ns
//     on Cyclone V (namco22 measurements), so MIN_GAP = 3 (31 ns) is safe
//     at 96.768 MHz and MIN_GAP = 2 is not. Peak step rate: 32.256 MHz.
//   * data_in (bc_rdata) and clkena come from nb1_main_bus/this module's
//     registers through short logic: ordinary single-cycle paths.
//   * IPL (M9) enters the kernel only from kin_ipl, which changes only on an
//     enable edge, exactly like a kernel register (multicycle kin_ipl ->
//     kernel = MIN_GAP in NB1.sdc).
//
// Interrupts (M9, docs/M9_RESEARCH.md section 5): ipl is the requested level
// (0 = none) from nb1_main_irq. TG68K samples it at instruction boundaries,
// runs one interrupt-acknowledge read cycle (FC = 111, address
// $FFFFFFF0 | level << 1) and, with IPL_autovector = 1, ignores its data and
// uses autovector 24 + level through the VBR (68020 format-0 frame). bc_fc
// exposes the function code so nb1_main_bus can answer that cycle as an IACK
// instead of an unmapped read.
//
// SR-2 scheduler [IMPLEMENTATION-DECISION, docs/M3_RESEARCH.md section 5]:
//   * ce_cpu (24.192 MHz, one pulse per 4 clk_sys) earns one credit = one
//     68EC020 clock of elapsed time. One TG68K step spends one credit.
//   * A step is taken when: the current bus cycle is complete (or there is
//     none), a credit is available, and MIN_GAP has elapsed.
//   * While a memory access stalls the CPU, credits accumulate. Afterwards
//     the CPU runs up to 4/3 of the nominal rate (one step per MIN_GAP
//     cycles) until the backlog is gone: that is the catch-up.
//   * The backlog is capped at CREDIT_MAX (32 steps = 1.32 us): the CPU is
//     never ahead of real time (steps <= credits earned) and never more than
//     CREDIT_MAX clocks behind while catching up. Credits beyond the cap are
//     counted in lost_credits: time the CPU could not recover. That counter
//     is the loud SR-2 deficit measurement (overlay + bench).
//   * Catch-up only compresses the spacing of the CPU's own steps. Every bus
//     cycle is still issued once, in program order, after the previous one
//     completed, so no bus ordering changes.
// ---------------------------------------------------------------------------
module nb1_cpu #(
    parameter int MIN_GAP      = 3,
    parameter int SAMPLE_DELAY = 2,
    parameter int CREDIT_MAX   = 32
) (
    input  wire        clk_sys,
    input  wire        reset,          // hold the CPU in reset (active high)
    input  wire        ce_cpu,         // nominal 68EC020 clock (time base)
    input  wire        frame_tick,     // one pulse per video frame (statistics)
    input  wire [2:0]  ipl,            // M9: requested interrupt level, 0 = none

    // bus cycle towards nb1_main_bus
    output wire        bc_start,       // 1-cycle pulse: bc_* hold a new bus cycle
    output wire [23:1] bc_addr,
    output wire [1:0]  bc_type,        // 00 fetch, 10 read, 11 write
    output wire [2:0]  bc_fc,          // M9: 68k function code (111 = interrupt acknowledge)
    output wire        bc_uds,         // byte lane D15-8 (even address), active high
    output wire        bc_lds,         // byte lane D7-0  (odd address),  active high
    output wire [15:0] bc_wdata,
    input  wire        bc_done,        // cycle serviced (combinational in the bc_start cycle)
    input  wire [15:0] bc_rdata,       // read data, valid while bc_done

    // status / debug (all registered)
    output reg         running = 1'b0,        // out of reset
    output reg  [23:0] fetch_pc = '0,          // address of the most recent opcode fetch
    output reg  [31:0] vec_ssp = '0,           // first longword read from $000000 after reset
    output reg  [31:0] vec_pc  = '0,           // first longword read from $000004 after reset
    output reg  [1:0]  vec_seen = '0,          // [0] SSP, [1] PC captured
    output reg  [31:0] steps = '0,             // TG68K steps since reset
    output reg  [31:0] bus_cycles = '0,        // bus cycles completed since reset
    output reg  [23:0] steps_last_frame = '0,  // steps in the last complete frame (nominal 405504)
    output reg  [23:0] earned_last_frame = '0, // ce_cpu pulses in that frame (405504)
    output reg  [31:0] lost_credits = '0,      // SR-2: credits dropped at CREDIT_MAX
    output reg  [15:0] max_mem_stall = '0,     // longest wait for one bus cycle (clk_sys)
    output reg  [31:0] mem_stall_cycles = '0,  // total cycles waiting for the bus
    // M22 predecode (nb1_main_bus PREDECODE): the kcap_* load strobe and the kernel address it loads.
    // pd_addr is the combinational kernel output; it may only be sampled on pd_sample, exactly like
    // kcap_addr (the SAMPLE_DELAY multicycle, NB1.sdc).
    output wire        pd_sample,
    output wire [23:1] pd_addr,
    output wire [3:0]  cacr,
    output wire [31:0] vbr,
    output reg  [5:0]  credit = '0
);
    initial begin
        if (MIN_GAP < SAMPLE_DELAY + 1) $error("nb1_cpu: MIN_GAP must be > SAMPLE_DELAY");
    end

    // ------------------------------------------------------------------ kernel
    wire        clkena;
    wire [31:0] k_addr;
    wire [15:0] k_dout;
    wire        k_nwr, k_nuds, k_nlds;
    wire [1:0]  k_state;
    wire [2:0]  k_fc;
    // M9: the interrupt level as the kernel sees it. It changes only on an
    // enable edge (see the timing contract above), so a level raised or
    // withdrawn between two steps is seen from the following step on.
    // Declared before the kernel for ModelSim 10.5b (no forward references).
    reg  [2:0]  kin_ipl = 3'd0;
    wire [3:0]  k_cacr;
    wire [31:0] k_vbr;

    // Generics as in Minimig-AGA_MiSTer / Arcade_namco22_MiSTer (switchable
    // features, CPU input fixed at 68020).
    TG68KdotC_Kernel #(
        .SR_Read(2), .VBR_Stackframe(2), .extAddr_Mode(2),
        .MUL_Mode(2), .DIV_Mode(2), .BitField(2),
        .BarrelShifter(1), .MUL_Hardware(1)
    ) cpu (
        .clk(clk_sys),
        .nReset(~reset),
        .clkena_in(clkena),
        .data_in(bc_rdata),
        .IPL(~kin_ipl),          // active-low pins; M9 main VBL (nb1_main_irq)
        .IPL_autovector(1'b1),   // NB-1 uses autovectors only [MAME-SOURCE]
        .berr(1'b0),
        .CPU(2'b11),             // 68020
        .addr_out(k_addr),
        .data_write(k_dout),
        .nWr(k_nwr),
        .nUDS(k_nuds),
        .nLDS(k_nlds),
        .busstate(k_state),
        .longword(),
        .nResetOut(),
        .FC(k_fc),
        .clr_berr(),
        .skipFetch(),
        .regin_out(),
        .CACR_out(k_cacr),
        .VBR_out(k_vbr)
    );

    // ------------------------------------------------------ bus-cycle capture
    // kcap_*: the only registers that sample kernel outputs (NB1.sdc matches
    // "*nb1_cpu*kcap_*" for the SAMPLE_DELAY multicycle).
    reg        kcap_start  = 1'b0;
    reg        kcap_valid  = 1'b0;     // a captured step is waiting for its enable
    reg        kcap_access = 1'b0;     // ... and it is a bus cycle (busstate != 01)
    reg [23:1] kcap_addr   = '0;
    reg [1:0]  kcap_type   = 2'b01;
    reg        kcap_uds    = 1'b0, kcap_lds = 1'b0;
    reg [15:0] kcap_wdata  = '0;
    reg [3:0]  kcap_cacr   = '0;
    reg [31:0] kcap_vbr    = '0;
    reg [2:0]  kcap_fc     = 3'b101;

    assign bc_start = kcap_start;
    assign bc_addr  = kcap_addr;
    assign bc_type  = kcap_type;
    assign bc_fc    = kcap_fc;
    assign bc_uds   = kcap_uds;
    assign bc_lds   = kcap_lds;
    assign bc_wdata = kcap_wdata;
    assign cacr     = kcap_cacr;
    assign vbr      = kcap_vbr;

    // age = clk_sys edges since the last enable edge (or since reset).
    reg [7:0] age = '0;
    wire gap_ok  = (age >= 8'(MIN_GAP));
    wire ready   = kcap_valid && (!kcap_access || bc_done);
    assign clkena = running && ready && gap_ok && (credit != 6'd0);
    assign pd_sample = !reset && !clkena && (age == 8'(SAMPLE_DELAY)) && !kcap_valid;   // = the kcap_* load below
    assign pd_addr   = k_addr[23:1];

    always @(posedge clk_sys) begin
        kcap_start <= 1'b0;
        if (reset) begin
            age        <= '0;
            kcap_valid <= 1'b0;
            kcap_access <= 1'b0;
            kin_ipl    <= 3'd0;
        end else begin
            if (clkena) kin_ipl <= ipl;
            age <= clkena ? 8'd1 : ((age == 8'hFF) ? age : age + 8'd1);
            if (clkena) begin
                kcap_valid <= 1'b0;
            end else if (age == 8'(SAMPLE_DELAY) && !kcap_valid) begin   // pd_sample (M22)
                kcap_valid  <= 1'b1;
                kcap_access <= (k_state != 2'b01);
                kcap_start  <= (k_state != 2'b01);
                kcap_addr   <= k_addr[23:1];
                kcap_type   <= k_state;
                kcap_fc     <= k_fc;
                kcap_uds    <= ~k_nuds;
                kcap_lds    <= ~k_nlds;
                kcap_wdata  <= k_dout;
                kcap_cacr   <= k_cacr;
                kcap_vbr    <= k_vbr;
            end
        end
    end

    // ------------------------------------------------------------- scheduler
    wire earn = running && ce_cpu;
    reg  [23:0] steps_frame = '0, earned_frame = '0;
    reg  [15:0] cur_stall = '0;

    always @(posedge clk_sys) begin
        if (reset) begin
            running          <= 1'b0;
            credit           <= '0;
            steps            <= '0;
            bus_cycles       <= '0;
            lost_credits     <= '0;
            max_mem_stall    <= '0;
            mem_stall_cycles <= '0;
            cur_stall        <= '0;
            steps_frame      <= '0;
            earned_frame     <= '0;
            steps_last_frame <= '0;
            earned_last_frame <= '0;
        end else begin
            running <= 1'b1;
            // credit += earn - clkena, saturating at CREDIT_MAX
            if (earn && !clkena) begin
                if (credit == 6'(CREDIT_MAX)) lost_credits <= lost_credits + 32'd1;
                else                          credit <= credit + 6'd1;
            end else if (!earn && clkena) begin
                credit <= credit - 6'd1;
            end
            if (clkena) begin
                steps <= steps + 32'd1;
                if (kcap_access) bus_cycles <= bus_cycles + 32'd1;
            end
            // memory stall = a captured bus cycle that the bus has not finished
            if (kcap_valid && kcap_access && !bc_done) begin
                cur_stall        <= cur_stall + 16'd1;
                mem_stall_cycles <= mem_stall_cycles + 32'd1;
                if (cur_stall + 16'd1 > max_mem_stall) max_mem_stall <= cur_stall + 16'd1;
            end else begin
                cur_stall <= '0;
            end
            // per-frame effective rate
            if (frame_tick) begin
                steps_last_frame  <= steps_frame + (clkena ? 24'd1 : 24'd0);
                earned_last_frame <= earned_frame + (earn ? 24'd1 : 24'd0);
                steps_frame       <= '0;
                earned_frame      <= '0;
            end else begin
                steps_frame  <= steps_frame + (clkena ? 24'd1 : 24'd0);
                earned_frame <= earned_frame + (earn ? 24'd1 : 24'd0);
            end
        end
    end

    // ----------------------------------------------------------- debug latches
    // Reset vectors as the CPU actually read them (whatever TG68K's reset
    // microcode does, the longwords at $0 and $4 arrive as two word reads).
    always @(posedge clk_sys) begin
        if (reset) begin
            vec_seen <= '0;
            fetch_pc <= '0;
        end else if (clkena && kcap_access) begin
            if (kcap_type == 2'b00) fetch_pc <= {kcap_addr, 1'b0};
            if (kcap_type != 2'b11 && kcap_addr[23:3] == '0 && !vec_seen[kcap_addr[2]]) begin
                if (kcap_addr[1] == 1'b0) begin
                    if (kcap_addr[2]) vec_pc[31:16]  <= bc_rdata; else vec_ssp[31:16] <= bc_rdata;
                end else begin
                    if (kcap_addr[2]) begin vec_pc[15:0]  <= bc_rdata; vec_seen[1] <= 1'b1; end
                    else              begin vec_ssp[15:0] <= bc_rdata; vec_seen[0] <= 1'b1; end
                end
            end
        end
    end
endmodule
