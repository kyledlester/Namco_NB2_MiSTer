// Namco NB-1 MiSTer core -- C75 sound/IO MCU (M8).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The C75 is a Mitsubishi M37702M2 with Namco's 16 KiB internal BIOS
// (MAME namcomcu.cpp: NAMCO_C75 = m37702m2_device, ROM c75.bin), clocked at
// 16.128 MHz = XTAL/3 (namconb1.cpp NAMCO_C75(..., 48.384_MHz_XTAL / 3)).
// This module is the NB-1 board glue around the vendored M37702 core and
// M37702M2 peripheral block (rtl/vendor/m37702, from the owner's NA-1 core);
// see docs/M8_RESEARCH.md and docs/M8_IMPLEMENTATION.md.
//
// Address map [MAME-SOURCE m37702m2_device::map + namconb1 namcoc75_am]:
//   $000000-$00007F  SFRs (na1_c69_sfr: ports, timers, A-D, interrupts)
//   $000080-$00027F  internal RAM, 512 bytes (on-chip; not cleared by reset)
//   $002000-$002FFF  C352 registers (M15): the c352_* port (nb1_c352; held
//                    request, one-cycle ack, word offset = (address - $2000) / 2)
//   $004000-$00BFFF  shared RAM, 16K x 16 (the 68020's $200000-$207FFF)
//   $00C000-$00FFFF  internal ROM (c75.bin, captured from the MRA stream)
//   $200000-$27FFFF  data ROM c75data (SDRAM region C75DATA, via a 2-line
//                    8-byte read buffer on an nb1_memory client port)
//   anything else    unmapped: reads $0000, writes ignored (MAME unmapped;
//                    the BIOS writes $150000 every INT2), counted
//
// Shared RAM lanes [MAME-SOURCE share_r / C75 .ram() on the same u16 array]:
// the 16-bit word is the same value on both sides. The C75 is little-endian,
// so its even byte is D7:0 (be[0]) = the 68020's odd byte; C75 byte address
// = ($4000 + 68020 offset) ^ 1. No data swap.
//
// Ports [MAME-SOURCE namconb1 port6_r/port6_w/port7_r, dac_bit_r]: P6 reads
// back what was last written (data & dir); P7 = {P4, MISC, P1, P2, $FF...}
// selected by P6[7:4] = 0,2,4,6 (else $FF); A-D channel n = $0080 when its
// P3 bit is set, else 0 (an7..an0 <- P3 bits 7,3,2,1,0,4,5,6); every other
// port input is unbound ($FF). The board inputs arrive active low (MAME
// IP_ACTIVE_LOW); with nothing pressed every one is $FF.
//
// Interrupts: INT0 / INT2 are external pulses (nb1_c75_irq); INT1 is not
// connected in MAME. Internal sources (timers, A-D) are in the SFR block.
//
// Bus contract of the core (na1_m37702): bus_req held with we/addr/be/wdata
// until bus_ack (level), then withdrawn for >= 1 cycle. Every internal
// target answers one clk_sys after the request; a data-ROM miss answers one
// clk_sys after its line arrives.
module nb1_c75 #(
    parameter bit COUNT_STOP = 1'b1     // M19: clearing a count-start bit stops that timer (see the SFR instance)
) (
    input  wire        clk_sys,
    input  wire        reset,            // 1 = C75 held (halted); release = reset pulse end
    input  wire        ce_c75,           // 16.128 MHz input-clock tick

    // internal ROM load port (MRA stream, region C75BIOS)
    input  wire        irom_we,
    input  wire [12:0] irom_waddr,
    input  wire [15:0] irom_wdata,

    // shared RAM, port B of the dual-port array in nb1_main_bus
    output wire        sh_en,
    output wire        sh_we,
    output wire [13:0] sh_addr,
    output wire [15:0] sh_wdata,
    output wire [1:0]  sh_be,            // [1] D15:8 (C75 odd byte), [0] D7:0 (C75 even byte)
    input  wire [15:0] sh_rdata,

    // nb1_memory client (region C75DATA line reads only)
    output reg         mreq_valid = 1'b0,
    input  wire        mreq_ready,
    output wire [3:0]  mreq_region,
    output reg  [24:0] mreq_offset = '0,
    output wire        mreq_we,
    output wire [31:0] mreq_wdata,
    output wire [3:0]  mreq_be,
    input  wire        mrsp_valid,
    input  wire [63:0] mrsp_line,
    input  wire        mrsp_err,

    // C352 register port (M15, nb1_c352)
    output wire        c352_req,
    output wire        c352_we,
    output wire [10:0] c352_addr,
    output wire [1:0]  c352_be,
    output wire [15:0] c352_wdata,
    input  wire        c352_ack,
    input  wire [15:0] c352_rdata,

    // external interrupt pulses
    input  wire        irq0_in,
    input  wire        irq2_in,

    // board inputs, active low (MAME P1, P2, P3, P4, MISC)
    input  wire [7:0]  in_p1,
    input  wire [7:0]  in_p2,
    input  wire [7:0]  in_p3,
    input  wire [7:0]  in_p4,
    input  wire [7:0]  in_misc,

    // diagnostics
    output wire        running,          // released and executing
    output reg  [23:0] pc = '0,          // address of the last instruction started
    output wire [31:0] instr_count,
    output wire [15:0] overrun_count,    // instructions whose bus waits exceeded MAME's cycle count
    output reg  [15:0] int0_count = '0,  // INT0 interrupts taken
    output reg  [15:0] int2_count = '0,
    output reg  [31:0] drom_reads = '0,  // data-ROM bus cycles
    output reg  [31:0] drom_misses = '0, // data-ROM line fills
    output reg  [15:0] max_drom_wait = '0, // longest data-ROM fill, clk_sys
    output reg  [15:0] c352_writes = '0,
    output reg  [15:0] unmapped_acc = '0,
    output reg  [7:0]  hs_sets = '0,     // C75 writes to $A000 with bit 7 set (the boot handshake)
    output reg  [15:0] hs_a000 = '0,     // last word the C75 wrote to $A000
    output reg  [15:0] hs_a0a0 = '0,     // last word the C75 wrote to $A0A0 (its copy of the 68020's $A030 answer)

    // bench observation of the core bus
    output wire        bus_req_o,
    output wire        bus_we_o,
    output wire [23:0] bus_addr_o,
    output wire [1:0]  bus_be_o,
    output wire [15:0] bus_wdata_o,
    output wire        bus_ack_o,
    output wire [15:0] bus_rdata_o,
    output wire        irq_ack_o,
    output wire [4:0]  irq_ack_line_o
);
    import nb1_mem_pkg::*;

    // ------------------------------------------------------------ core
    wire        bus_req, bus_we, bus_ack;
    wire [23:0] bus_addr;
    wire [1:0]  bus_be;
    wire [15:0] bus_wdata, bus_rdata;
    wire        irq_take, irq_ack, instr_commit, instr_start, flag_i, cpu_stopped;
    wire [4:0]  irq_line, irq_ack_line;
    wire [2:0]  irq_pri, ipl;
    wire [31:0] cycle_count;

    na1_m37702 cpu (
        .clk_sys(clk_sys), .reset(reset), .ce_mcu(ce_c75),
        .bus_req(bus_req), .bus_we(bus_we), .bus_addr(bus_addr), .bus_be(bus_be), .bus_wdata(bus_wdata),
        .bus_ack(bus_ack), .bus_rdata(bus_rdata),
        .irq_take(irq_take), .irq_line(irq_line), .irq_pri(irq_pri),
        .irq_ack(irq_ack), .irq_ack_line(irq_ack_line),
        .flag_i_out(flag_i), .ipl_out(ipl), .instr_commit(instr_commit),
        .instr_start(instr_start), .instr_count(instr_count), .cycle_count(cycle_count),
        .overrun_count(overrun_count), .stopped(cpu_stopped));

    assign running = !reset && !cpu_stopped;

    // ------------------------------------------------------------ decode
    wire sel_sfr  = (bus_addr[23:7] == 17'd0);
    wire sel_iram = (bus_addr >= 24'h000080) && (bus_addr <= 24'h00027F);
    wire sel_c352 = (bus_addr[23:12] == 12'h002);                       // $2000-$2FFF
    wire sel_shr  = (bus_addr >= 24'h004000) && (bus_addr <= 24'h00BFFF);
    wire sel_rom  = (bus_addr[23:14] == 10'h003);                        // $C000-$FFFF
    wire sel_drom = (bus_addr[23:19] == 5'b00100);                       // $200000-$27FFFF
    wire sel_none = !(sel_sfr | sel_iram | sel_c352 | sel_shr | sel_rom | sel_drom);

    // ------------------------------------------------------------ on-chip memories
    // internal RAM 256 x 16, internal ROM 8192 x 16 (M10K); byte-lane pattern as nb1_cpu_ram
    (* ramstyle = "M10K, no_rw_check" *) reg [1:0][7:0] iram[0:255];
    (* ramstyle = "M10K, no_rw_check" *) reg [15:0]      irom[0:8191];
    reg [15:0] iram_q = '0, irom_q = '0;
// synthesis translate_off
`ifndef SYNTHESIS
    initial begin
        // [IMPLEMENTATION] MAME's internal RAM starts zeroed; physical power-up [UNKNOWN].
        for (int i = 0; i < 256; i++)  iram[i] = '0;
        for (int i = 0; i < 8192; i++) irom[i] = '0;
    end
`endif
// synthesis translate_on
    wire [7:0] iram_a = bus_addr[8:1] - 8'h40;        // ($80..$27F)/2 - $40

    reg int_ack = 1'b0;
    wire drom_hit;
    wire int_sel = !sel_sfr && (sel_iram | sel_shr | sel_rom | sel_none | (sel_drom && drom_hit));
    wire first   = bus_req && !int_ack;               // the request's first cycle (writes act here)

    always @(posedge clk_sys) begin
        if (irom_we) irom[irom_waddr] <= irom_wdata;
        irom_q <= irom[bus_addr[13:1]];
        if (first && bus_we && sel_iram) begin
            if (bus_be[1]) iram[iram_a][1] <= bus_wdata[15:8];
            if (bus_be[0]) iram[iram_a][0] <= bus_wdata[7:0];
        end
        iram_q <= iram[iram_a];
        int_ack <= reset ? 1'b0 : (bus_req && int_sel && !int_ack);
    end

    // shared RAM port B: one access in the request's first cycle, data next cycle
    assign sh_en    = first && sel_shr;
    assign sh_we    = bus_we;
    assign sh_addr  = bus_addr[14:1] - 14'h2000;      // ($4000..$BFFF)/2 - $2000
    assign sh_wdata = bus_wdata;
    assign sh_be    = bus_be;

    // ------------------------------------------------------------ data ROM buffer
    // Two direct-mapped 8-byte lines (index A3), the controller's native burst
    // (nb1_memory rsp_line, byte A+0 in bits 63:56). The region is read-only
    // while the C75 runs (downloads hold it in reset), so lines never go stale;
    // they are emptied by reset.
    assign mreq_region = REG_C75DATA;
    assign mreq_we     = 1'b0;
    assign mreq_wdata  = '0;
    assign mreq_be     = 4'b1111;

    (* ramstyle = "logic" *) reg [63:0] line_data [0:1];   // M16: tiny; kept out of M10K
    reg [18:4] line_tag  [0:1];
    reg [1:0]  line_valid = 2'b00;
    reg        filling = 1'b0;
    // Accepted request not yet answered. Deliberately NOT cleared by reset:
    // a halt/restart during a fill must not start a new fill until the old
    // response has gone by, or that stale line would be taken as the new one
    // (nb1_memory contract: ignore rsp_valid with nothing outstanding).
    reg        drom_pending = 1'b0;
    wire       fill_start = bus_req && sel_drom && !drom_hit && !filling && !drom_pending;
    reg [15:0] fill_wait = '0;
    wire       li = bus_addr[3];
    assign drom_hit = line_valid[li] && (line_tag[li] == bus_addr[18:4]);
    wire [63:0] ld = line_data[li];
    // C75 little-endian word at even address a: {byte a+1, byte a}
    reg [15:0] drom_word;
    always_comb case (bus_addr[2:1])
        2'd0: drom_word = {ld[55:48], ld[63:56]};
        2'd1: drom_word = {ld[39:32], ld[47:40]};
        2'd2: drom_word = {ld[23:16], ld[31:24]};
        default: drom_word = {ld[7:0], ld[15:8]};
    endcase

    always @(posedge clk_sys) begin
        if (mreq_valid && mreq_ready) drom_pending <= 1'b1;
        else if (mrsp_valid)          drom_pending <= 1'b0;
    end

    always @(posedge clk_sys) begin
        if (reset) begin
            line_valid <= 2'b00;
            filling    <= 1'b0;
            mreq_valid <= 1'b0;
        end else begin
            if (fill_start) begin
                filling     <= 1'b1;
                mreq_valid  <= 1'b1;
                mreq_offset <= {6'd0, bus_addr[18:3], 3'b000};
                fill_wait   <= '0;
            end
            if (mreq_valid && mreq_ready) mreq_valid <= 1'b0;
            if (filling) fill_wait <= fill_wait + 16'd1;
            if (filling && mrsp_valid) begin
                filling <= 1'b0;
                // rsp_err cannot happen (the window is exactly the region); a
                // failed fill would leave the line invalid and retry.
                if (!mrsp_err) begin
                    line_data[li]  <= mrsp_line;
                    line_tag[li]   <= bus_addr[18:4];
                    line_valid[li] <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------ SFR block + board glue
    wire        sfr_ack;
    wire [15:0] sfr_rdata;
    wire [7:0]  port_in  [0:8];
    wire [7:0]  port_reg [0:8];
    wire [7:0]  port_dir [0:8];
    wire [8:0]  port_wr;
    wire [15:0] an_in [0:7];

    reg [7:0] mcu_port6 = 8'h00;
    always @(posedge clk_sys)
        if (reset) mcu_port6 <= 8'h00;
        else if (port_wr[6]) mcu_port6 <= port_reg[6] & port_dir[6];

    wire [7:0] p7_mux = (mcu_port6[7:4] == 4'h0) ? in_p4 :
                        (mcu_port6[7:4] == 4'h2) ? in_misc :
                        (mcu_port6[7:4] == 4'h4) ? in_p1 :
                        (mcu_port6[7:4] == 4'h6) ? in_p2 : 8'hFF;
    assign port_in[0] = 8'hFF; assign port_in[1] = 8'hFF; assign port_in[2] = 8'hFF;
    assign port_in[3] = 8'hFF; assign port_in[4] = 8'hFF; assign port_in[5] = 8'hFF;
    assign port_in[6] = mcu_port6;
    assign port_in[7] = p7_mux;
    assign port_in[8] = 8'hFF;

    // an<ch> <- P3 bit: ch7..ch0 = bits 7,3,2,1,0,4,5,6 (dac_bit_r<Bit>)
    localparam [23:0] AN_BIT = {3'd7, 3'd3, 3'd2, 3'd1, 3'd0, 3'd4, 3'd5, 3'd6};
    genvar g;
    generate for (g = 0; g < 8; g++) begin : an
        assign an_in[g] = in_p3[AN_BIT[3*g +: 3]] ? 16'h0080 : 16'h0000;
    end endgenerate

    // M19: COUNT_STOP -- clearing a count-start bit stops that timer (M37702 datasheet). The Nebulas Ray
    // driver restarts timer A0 in its 60 Hz INT0 handler and clears the bit after A0's 8.33 ms interrupt;
    // without the stop, A0's second expiry beat the restart and the sound driver ticked at 120 Hz
    // (docs/M19_RESEARCH.md).
    na1_c69_sfr #(.MAME_EARLY(1), .COUNT_STOP(COUNT_STOP)) sfr (
        .clk_sys(clk_sys), .reset(reset), .ce_mcu(ce_c75),
        .req(bus_req && sel_sfr), .we(bus_we), .addr(bus_addr[6:0]), .be(bus_be), .wdata(bus_wdata),
        .ack(sfr_ack), .rdata(sfr_rdata), .instr_commit(instr_commit),
        .port_in(port_in), .port_reg(port_reg), .port_dir(port_dir), .port_wr(port_wr), .an_in(an_in),
        .irq0_in(irq0_in), .irq1_in(1'b0), .irq2_in(irq2_in),
        .flag_i(flag_i), .ipl(ipl), .irq_take(irq_take), .irq_line(irq_line), .irq_pri(irq_pri),
        .irq_ack(irq_ack), .irq_ack_line(irq_ack_line),
        .sim_force_valid(1'b0), .sim_force_line(5'd0), .sim_inject_only(1'b0));

    // ------------------------------------------------------------ response
    assign c352_req   = bus_req && sel_c352;
    assign c352_we    = bus_we;
    assign c352_addr  = bus_addr[11:1];
    assign c352_be    = bus_be;
    assign c352_wdata = bus_wdata;
    assign bus_ack   = sel_sfr ? sfr_ack : sel_c352 ? c352_ack : int_ack;
    assign bus_rdata = sel_sfr  ? sfr_rdata :
                       sel_c352 ? c352_rdata :
                       sel_iram ? iram_q :
                       sel_shr  ? sh_rdata :
                       sel_rom  ? irom_q :
                       sel_drom ? drom_word : 16'h0000;       // unmapped read 0

    // ------------------------------------------------------------ diagnostics
    reg want_pc = 1'b0;
    always @(posedge clk_sys) begin
        if (reset) begin
            want_pc <= 1'b0; int0_count <= '0; int2_count <= '0; drom_reads <= '0; drom_misses <= '0;
            max_drom_wait <= '0; c352_writes <= '0; unmapped_acc <= '0; hs_sets <= '0;
            hs_a000 <= '0; hs_a0a0 <= '0;
        end else begin
            if (instr_start) want_pc <= 1'b1;
            if (want_pc && first && !bus_we) begin       // the opcode fetch of the new instruction
                want_pc <= 1'b0;
                pc      <= bus_addr + {23'd0, (bus_be == 2'b10)};
            end
            if (irq_ack && irq_ack_line == 5'd19) int0_count <= int0_count + 16'd1;
            if (irq_ack && irq_ack_line == 5'd17) int2_count <= int2_count + 16'd1;
            if (first && sel_drom && drom_hit) drom_reads <= drom_reads + 32'd1;
            if (fill_start) drom_misses <= drom_misses + 32'd1;
            if (filling && mrsp_valid && fill_wait > max_drom_wait) max_drom_wait <= fill_wait;
            if (c352_req && c352_we && c352_ack) c352_writes <= c352_writes + 16'd1;
            if (first && sel_none) unmapped_acc <= unmapped_acc + 16'd1;
            if (first && bus_we && sel_shr && bus_addr == 24'h00A000) begin
                if (bus_be[0]) hs_a000[7:0]  <= bus_wdata[7:0];
                if (bus_be[1]) hs_a000[15:8] <= bus_wdata[15:8];
                if (bus_be[0] && bus_wdata[7] && hs_sets != 8'hFF) hs_sets <= hs_sets + 8'd1;
            end
            if (first && bus_we && sel_shr && bus_addr == 24'h00A0A0) begin
                if (bus_be[0]) hs_a0a0[7:0]  <= bus_wdata[7:0];
                if (bus_be[1]) hs_a0a0[15:8] <= bus_wdata[15:8];
            end
        end
    end

    assign bus_req_o = bus_req; assign bus_we_o = bus_we; assign bus_addr_o = bus_addr;
    assign bus_be_o = bus_be; assign bus_wdata_o = bus_wdata; assign bus_ack_o = bus_ack;
    assign bus_rdata_o = bus_rdata; assign irq_ack_o = irq_ack; assign irq_ack_line_o = irq_ack_line;
endmodule
