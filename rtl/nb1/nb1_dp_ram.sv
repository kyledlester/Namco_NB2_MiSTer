// Namco NB-1 MiSTer core -- true dual-port byte-enabled RAM (M8).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Two independent 16-bit ports on one array, each with byte enables and one
// clk_sys of read latency, both in clk_sys. Used for the NB-1 shared RAM:
// port A = 68EC020 (nb1_main_bus), port B = C75 (nb1_c75).
// M24 collision rules (Quartus infers these arrays with mixed-port
// read-during-write = DONT_CARE, where an M10K returns undefined data):
//  * a port that reads a byte lane the other port writes in the same clk_sys
//    gets the newly written byte (bypass below), never the M10K's output;
//  * both ports writing the same byte lane in the same clk_sys: port A wins
//    (port B's write of that lane is dropped), i.e. B then A.
// Either way the result is one of the two orders a sequential model (MAME)
// can produce. Found as the cause of garbled C75 sound commands: the C75
// polls the shared-RAM command slots while the 68020 writes them.
// Read-during-write on the same port returns old data (no_rw_check: no
// client uses it).
module nb1_dp_ram #(parameter int WORDS = 16384, parameter int AW = 14) (
    input  wire          clk_sys,
    input  wire          en_a,
    input  wire          we_a,
    input  wire [AW-1:0] addr_a,
    input  wire [15:0]   wdata_a,
    input  wire [1:0]    be_a,
    output reg  [15:0]   rdata_a,
    input  wire          en_b,
    input  wire          we_b,
    input  wire [AW-1:0] addr_b,
    input  wire [15:0]   wdata_b,
    input  wire [1:0]    be_b,
    output reg  [15:0]   rdata_b
);
    // One 8-bit true-dual-port array per byte lane: Quartus 17.0 Standard does
    // not infer a byte-enabled true-dual-port RAM (Error 276003 with the
    // packed-lane coding), but infers two plain TDP arrays; the lane's write
    // enable is we & be. hi = D15:8 (68020 even byte), lo = D7:0.
    (* ramstyle = "M10K, no_rw_check" *) reg [7:0] hi[0:WORDS-1];
    (* ramstyle = "M10K, no_rw_check" *) reg [7:0] lo[0:WORDS-1];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < WORDS; i++) begin hi[i] = '0; lo[i] = '0; end
`endif
// synthesis translate_on
    // M24: same-address collisions between the ports (see the header)
    wire       same  = (addr_a == addr_b);
    wire [1:0] wr_a  = {2{en_a && we_a}} & be_a;
    wire [1:0] wr_b  = ({2{en_b && we_b}} & be_b) & ~({2{en_a && same}} & wr_a);   // A wins
    reg  [1:0] byp_a = '0, byp_b = '0;     // this read sees the other port's write
    reg [15:0] bw_a = '0, bw_b = '0;       // that write's data
    reg [15:0] q_a = '0, q_b = '0;         // array outputs
    // updated only with the port's own enable, so an idle port keeps showing its last read
    always @(posedge clk_sys) begin
        if (en_a) begin byp_a <= {2{en_b && same}} & wr_b; bw_a <= wdata_b; end
        if (en_b) begin byp_b <= {2{en_a && same}} & wr_a; bw_b <= wdata_a; end
    end
    always @* begin
        rdata_a[15:8] = byp_a[1] ? bw_a[15:8] : q_a[15:8];
        rdata_a[7:0]  = byp_a[0] ? bw_a[7:0]  : q_a[7:0];
        rdata_b[15:8] = byp_b[1] ? bw_b[15:8] : q_b[15:8];
        rdata_b[7:0]  = byp_b[0] ? bw_b[7:0]  : q_b[7:0];
    end
    always @(posedge clk_sys) if (en_a) begin
        if (wr_a[1]) hi[addr_a] <= wdata_a[15:8];
        q_a[15:8] <= hi[addr_a];
    end
    always @(posedge clk_sys) if (en_a) begin
        if (wr_a[0]) lo[addr_a] <= wdata_a[7:0];
        q_a[7:0] <= lo[addr_a];
    end
    always @(posedge clk_sys) if (en_b) begin
        if (wr_b[1]) hi[addr_b] <= wdata_b[15:8];
        q_b[15:8] <= hi[addr_b];
    end
    always @(posedge clk_sys) if (en_b) begin
        if (wr_b[0]) lo[addr_b] <= wdata_b[7:0];
        q_b[7:0] <= lo[addr_b];
    end
endmodule
