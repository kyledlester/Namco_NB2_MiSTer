// Namco NB-2 MiSTer core -- C355 sprite RAM, position and bank registers (M11).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// NB-2 copy of the NB-1 core's nb1_c355_ram (docs/PROVENANCE.md). One change: words that are not backed read $0000
// (NB-1: $FFFF). MAME's sprite RAM starts zeroed and Mach Breakers reads never-written words at $613000-$613FFF and
// $61F000-$61FFFF (MAME profile, docs/NB2_HARDWARE_REFERENCE.md); no game reads back an unbacked word it wrote.
//
// MAME [MAME-SOURCE namconb1.cpp maincpu_am, namco_c355spr.cpp]:
//   $600000-$61FFFF  sprite RAM, 64K words, read/write. Writes to page 1
//                    $610000-$611FFF and $614000-$6143FF are ALSO written to
//                    page 0 at -$10000 / -$12000 (the C355 reads page 0).
//   $620000-$620007  position registers, 4 words, read/write
//   $680000-$68000F  sprite tile bank, 8 words, plain RAM
// None has a reset (power-up zero).
//
// Storage [IMPLEMENTATION-DECISION, docs/M11_RESEARCH.md s.2 and s.8]: block
// RAM cannot hold 128 KiB, so only what the C355 reads and what the CPU
// reads back is backed:
//   page 0 words $0000-$07FF  attributes (256 sprites x 8)       2K words
//   page 0 words $1000-$13FF  display list + clip windows         1K words
//   page 0 words $2000-$7FFF  format table + tile table          24K words
//   page 1 words $A000-$A1FF  the page-1 list the CPU reads back  512 words
// Every other word is write-only here: writes are dropped (the C355 never
// reads them) and reads return $FFFF and are counted (`unbacked_reads`).
// Nebulas Ray's 90 s census: CPU reads only $614000-$6143FF; the C355 reads
// only the backed page-0 windows.
//
// Port A = CPU (one-cycle synchronous read like the other nb1_main_bus
// devices). Port B = the renderer's snapshot port, page-0 word address
// $0000-$7FFF, data one clk later ($0000 for unbacked words).
module nb2_c355_ram (
    input  wire         clk_sys,

    // CPU: sprite RAM ($600000-$61FFFF)
    input  wire         obj_access,
    input  wire         write,
    input  wire [16:1]  addr,         // byte offset in $600000-$61FFFF
    input  wire [15:0]  wdata,
    input  wire [1:0]   be,
    output wire [15:0]  obj_rdata,
    // CPU: position ($620000-$620007) and bank ($680000-$68000F)
    input  wire         pos_access,
    input  wire         bank_access,
    input  wire [3:1]   reg_addr,
    output reg  [15:0]  reg_rdata = 16'h0000,

    // renderer
    input  wire [14:0]  snap_addr,    // page-0 word
    output reg  [15:0]  snap_data = 16'h0000,
    output wire [63:0]  pos_out,      // {w3, w2, w1, w0}
    output wire [127:0] bank_out,     // {w7 .. w0}

    output reg  [31:0]  unbacked_reads = '0
);
    wire [15:0] w = addr[16:1];                        // word offset 0..$FFFF

    // CPU word -> page-0 word written (direct or mirror) and page-1 list word
    wire        pg1_attr = (w >= 16'h8000) && (w < 16'h9000);
    wire        pg1_list = (w >= 16'hA000) && (w < 16'hA200);
    wire [15:0] p0w = pg1_attr ? w - 16'h8000 : pg1_list ? w - 16'h9000 : w;
    wire        p0_ok = !w[15] || pg1_attr || pg1_list;

    function automatic [1:0] win(input [14:0] a);      // 1 attr, 2 list/clip, 3 tables, 0 none
        if (a < 15'h0800)                     win = 2'd1;
        else if (a >= 15'h1000 && a < 15'h1400) win = 2'd2;
        else if (a >= 15'h2000)               win = 2'd3;
        else                                  win = 2'd0;
    endfunction
    wire [1:0] cw = p0_ok ? win(p0w[14:0]) : 2'd0;
    wire [1:0] sw = win(snap_addr);

    // tables $2000-$7FFF split 16K ($4000-$7FFF) + 8K ($2000-$3FFF)
    wire [15:0] qa_attr, qa_list, qa_t16, qa_t8, qb_attr, qb_list, qb_t16, qb_t8, q_p1;
    wire        c_t16 = (cw == 2'd3) && p0w[14];
    wire        c_t8  = (cw == 2'd3) && !p0w[14];
    wire        s_t16 = (sw == 2'd3) && snap_addr[14];

    nb1_dp_ram #(.WORDS(2048), .AW(11)) attr_ram (
        .clk_sys(clk_sys), .en_a(obj_access && cw == 2'd1), .we_a(write), .addr_a(p0w[10:0]),
        .wdata_a(wdata), .be_a(be), .rdata_a(qa_attr),
        .en_b(1'b1), .we_b(1'b0), .addr_b(snap_addr[10:0]), .wdata_b(16'h0), .be_b(2'b00), .rdata_b(qb_attr));
    nb1_dp_ram #(.WORDS(1024), .AW(10)) list_ram (
        .clk_sys(clk_sys), .en_a(obj_access && cw == 2'd2), .we_a(write), .addr_a(p0w[9:0]),
        .wdata_a(wdata), .be_a(be), .rdata_a(qa_list),
        .en_b(1'b1), .we_b(1'b0), .addr_b(snap_addr[9:0]), .wdata_b(16'h0), .be_b(2'b00), .rdata_b(qb_list));
    nb1_dp_ram #(.WORDS(16384), .AW(14)) t16_ram (
        .clk_sys(clk_sys), .en_a(obj_access && c_t16), .we_a(write), .addr_a(p0w[13:0]),
        .wdata_a(wdata), .be_a(be), .rdata_a(qa_t16),
        .en_b(1'b1), .we_b(1'b0), .addr_b(snap_addr[13:0]), .wdata_b(16'h0), .be_b(2'b00), .rdata_b(qb_t16));
    nb1_dp_ram #(.WORDS(8192), .AW(13)) t8_ram (
        .clk_sys(clk_sys), .en_a(obj_access && c_t8), .we_a(write), .addr_a(p0w[12:0]),
        .wdata_a(wdata), .be_a(be), .rdata_a(qa_t8),
        .en_b(1'b1), .we_b(1'b0), .addr_b(snap_addr[12:0]), .wdata_b(16'h0), .be_b(2'b00), .rdata_b(qb_t8));
    // page-1 list (CPU read-back of $614000-$6143FF)
    nb1_cpu_ram #(.WORDS(512), .AW(9)) p1_list (
        .clk_sys(clk_sys), .en(obj_access && pg1_list), .we(write), .addr(w[8:0]),
        .wdata(wdata), .be(be), .rdata(q_p1));

    // CPU read data (registered selection, valid with the RAM data)
    reg [2:0] rsel = 3'd0;   // 0 none, 1 attr, 2 list, 3 t16, 4 t8, 5 page-1 list
    always @(posedge clk_sys) if (obj_access) begin
        rsel <= pg1_list ? 3'd5 : w[15] ? 3'd0 : (cw == 2'd1) ? 3'd1 : (cw == 2'd2) ? 3'd2 :
                c_t16 ? 3'd3 : c_t8 ? 3'd4 : 3'd0;
        if (!write && (w[15] ? !pg1_list : cw == 2'd0)) unbacked_reads <= unbacked_reads + 32'd1;
    end
    assign obj_rdata = (rsel == 3'd1) ? qa_attr : (rsel == 3'd2) ? qa_list : (rsel == 3'd3) ? qa_t16 :
                       (rsel == 3'd4) ? qa_t8 : (rsel == 3'd5) ? q_p1 : 16'h0000;

    // snapshot port data
    reg [1:0] ssel = 2'd0;
    reg       s16 = 1'b0;
    always @(posedge clk_sys) begin ssel <= sw; s16 <= s_t16; end
    always_comb snap_data = (ssel == 2'd1) ? qb_attr : (ssel == 2'd2) ? qb_list :
                            (ssel == 2'd3) ? (s16 ? qb_t16 : qb_t8) : 16'h0000;

    // position and bank registers (flops, byte lanes, read back)
    reg [15:0] pos  [0:3];
    reg [15:0] bank [0:7];
// synthesis translate_off
`ifndef SYNTHESIS
    initial begin
        for (int i = 0; i < 4; i++) pos[i] = 16'h0000;
        for (int i = 0; i < 8; i++) bank[i] = 16'h0000;
    end
`endif
// synthesis translate_on
    always @(posedge clk_sys) begin
        if (pos_access) begin
            if (write && be[1]) pos[reg_addr[2:1]][15:8] <= wdata[15:8];
            if (write && be[0]) pos[reg_addr[2:1]][7:0]  <= wdata[7:0];
            reg_rdata <= pos[reg_addr[2:1]];
        end else if (bank_access) begin
            if (write && be[1]) bank[reg_addr][15:8] <= wdata[15:8];
            if (write && be[0]) bank[reg_addr][7:0]  <= wdata[7:0];
            reg_rdata <= bank[reg_addr];
        end
    end
    genvar g;
    generate
        for (g = 0; g < 4; g++) begin : g_pos  assign pos_out[g*16 +: 16]  = pos[g];  end
        for (g = 0; g < 8; g++) begin : g_bank assign bank_out[g*16 +: 16] = bank[g]; end
    endgenerate
endmodule
