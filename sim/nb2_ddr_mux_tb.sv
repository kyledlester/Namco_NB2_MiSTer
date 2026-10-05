// Namco NB-2 MiSTer core -- unit bench for rtl/nb2/nb2_ddr_mux.sv on the DDR3 model (random BUSY, random read
// latency). Master A: random reads and writes (held until accepted, as nb2_obj_store), every read checked against
// a shadow copy, in order. Master B: screen_rotate's pattern, a write pulse every B_GAP clocks regardless of BUSY
// (8 = its fastest rate, scandoubled output), half-word selects F0/0F. At the end every B word must be in memory
// and no B write may have been dropped.
// Plusargs: +cycles=<n> (default 400000), +bgap=<n> (default 8), +apct=<A command chance %> (default 60).
`timescale 1ns/1ps
module nb2_ddr_mux_tb;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    localparam int BASE = 29'h6400000;
    localparam int A_WORDS = 4096;                  // A area: words BASE .. BASE+4095
    localparam int B_OFF = 8192, B_WORDS = 8192;    // B area: words BASE+8192 ..

    // A master
    reg        a_rd = 1'b0, a_we = 1'b0;
    reg [28:0] a_addr = '0;
    reg [63:0] a_din = '0;
    reg [7:0]  a_be = '0;
    wire       a_busy;
    // B master
    reg        b_we = 1'b0;
    reg [28:0] b_addr = '0;
    reg [63:0] b_din = '0;
    reg [7:0]  b_be = '0;

    wire        busy, rd, we, dout_ready;
    wire [7:0]  burstcnt, be;
    wire [28:0] addr;
    wire [63:0] din, dout;
    wire [15:0] b_over;

    nb2_ddr_mux dut (
        .clk(clk),
        .a_rd(a_rd), .a_we(a_we), .a_addr(a_addr), .a_din(a_din), .a_be(a_be), .a_busy(a_busy),
        .b_we(b_we), .b_addr(b_addr), .b_din(b_din), .b_be(b_be),
        .DDRAM_BUSY(busy), .DDRAM_BURSTCNT(burstcnt), .DDRAM_ADDR(addr), .DDRAM_DIN(din), .DDRAM_BE(be),
        .DDRAM_WE(we), .DDRAM_RD(rd), .b_overflows(b_over));
    ddram_model #(.BASE_WORD(BASE), .WORDS(B_OFF + B_WORDS), .UNWRITTEN(64'd0)) ddr (
        .clk(clk), .busy(busy), .burstcnt(burstcnt), .addr(addr), .dout(dout), .dout_ready(dout_ready),
        .rd(rd), .din(din), .be(be), .we(we));

    int unsigned seed = 99;
    function automatic int unsigned rnd();
        seed = seed * 1664525 + 1013904223;
        return seed >> 4;
    endfunction

    bit [63:0] shadow [0:A_WORDS-1];
    bit [63:0] bexp [0:B_WORDS-1];
    bit [63:0] rq [$];                              // expected read data, in order
    int errors = 0, a_reads = 0, a_writes = 0, b_writes = 0, checked = 0;
    int cycles, bgap, apct;
    longint cyc = 0;

    // A: present a command, hold it until accepted (a_busy low), then maybe another next clock
    always @(posedge clk) begin
        cyc <= cyc + 1;
        if ((a_rd || a_we) && !a_busy) begin
            if (a_we) begin
                for (int k = 0; k < 8; k++) if (a_be[k]) shadow[a_addr - BASE][8*k +: 8] = a_din[8*k +: 8];
                a_writes++;
            end else begin
                rq.push_back(shadow[a_addr - BASE]);
                a_reads++;
            end
            a_rd <= 1'b0; a_we <= 1'b0;
        end
        if ((!(a_rd || a_we) || !a_busy) && cyc < cycles - 2000 && (rnd() % 100) < apct) begin
            automatic int w = rnd() % A_WORDS;
            a_addr <= BASE + w;
            if (rnd() % 3 == 0) begin
                a_we <= 1'b1; a_rd <= 1'b0;
                a_din <= {rnd(), rnd()};
                a_be <= 8'(rnd()) | 8'h01;
            end else begin
                a_rd <= 1'b1; a_we <= 1'b0;
            end
        end
        if (dout_ready) begin
            if (rq.size() == 0) begin errors++; $display("ERROR: read data with no read pending"); end
            else begin
                if (dout !== rq[0]) begin
                    errors++;
                    if (errors < 10) $display("ERROR: read %0d: got %h, expected %h", checked, dout, rq[0]);
                end
                void'(rq.pop_front());
                checked++;
            end
        end
    end

    // B: one write pulse every bgap clocks, ignoring BUSY (screen_rotate)
    int bcnt = 0;
    always @(posedge clk) begin
        b_we <= 1'b0;
        bcnt <= (bcnt == bgap - 1) ? 0 : bcnt + 1;
        if (bcnt == 0 && cyc < cycles - 2000) begin
            automatic int w = rnd() % B_WORDS;
            automatic bit hi = rnd() % 2;
            automatic bit [31:0] px = rnd();
            b_we   <= 1'b1;
            b_addr <= BASE + B_OFF + w;
            b_din  <= {px, px};
            b_be   <= hi ? 8'hF0 : 8'h0F;
            if (hi) bexp[w][63:32] = px; else bexp[w][31:0] = px;
            b_writes++;
        end
    end

    initial begin
        if (!$value$plusargs("cycles=%d", cycles)) cycles = 400000;
        if (!$value$plusargs("bgap=%d", bgap)) bgap = 8;
        if (!$value$plusargs("apct=%d", apct)) apct = 60;
        wait (cyc >= cycles);
        if (rq.size() != 0) begin errors++; $display("ERROR: %0d reads never answered", rq.size()); end
        for (int w = 0; w < B_WORDS; w++)
            if (ddr.mem[B_OFF + w] !== bexp[w]) begin
                errors++;
                if (errors < 20) $display("ERROR: B word %0d: memory %h, expected %h", w, ddr.mem[B_OFF + w], bexp[w]);
            end
        $display("A: %0d reads (%0d checked), %0d writes; B: %0d writes, %0d dropped; model errors %0d",
                 a_reads, checked, a_writes, b_writes, b_over, ddr.errors);
        if (errors == 0 && b_over == 0 && ddr.errors == 0) $display("PASS nb2_ddr_mux_tb");
        else $display("FAIL nb2_ddr_mux_tb (%0d errors)", errors);
        $finish;
    end
endmodule
