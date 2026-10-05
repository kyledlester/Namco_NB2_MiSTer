// Namco NB-2 MiSTer core -- DDR3 port shared by the sprite store and the framework's screen_rotate.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Master A: nb2_obj_store (Avalon-MM: holds RD/WE until BUSY is low; the only reader, so DDRAM_DOUT goes straight
// back to it). Master B: screen_rotate (sys/arcade_video.v, Orientation Vertical CW/CCW): one single-beat write
// pulse per pixel that never waits for BUSY, so its writes are queued here (FIFO, depth 16) and written into the
// gaps between A's commands; when the queue reaches half full it goes first, so it never overflows (B writes at
// most one word per 8 clocks; a write takes one port cycle plus BUSY).
// A command on the port is never withdrawn: the grant changes only on a cycle where the port has no command or
// its command is accepted (BUSY low).
module nb2_ddr_mux (
    input  wire        clk,

    // A: sprite store
    input  wire        a_rd,
    input  wire        a_we,
    input  wire [28:0] a_addr,
    input  wire [63:0] a_din,
    input  wire [7:0]  a_be,
    output wire        a_busy,

    // B: screen_rotate (BURSTCNT 1, RD 0)
    input  wire        b_we,
    input  wire [28:0] b_addr,
    input  wire [63:0] b_din,
    input  wire [7:0]  b_be,

    // DDR3
    input  wire        DDRAM_BUSY,
    output wire [7:0]  DDRAM_BURSTCNT,
    output wire [28:0] DDRAM_ADDR,
    output wire [63:0] DDRAM_DIN,
    output wire [7:0]  DDRAM_BE,
    output wire        DDRAM_WE,
    output wire        DDRAM_RD,

    output reg  [15:0] b_overflows = '0       // diagnostics: B writes lost (queue full)
);
    assign DDRAM_BURSTCNT = 8'd1;

    // B queue. screen_rotate writes the same 32-bit pixel to both halves and selects one with BE (F0 / 0F), so an
    // entry is {addr, pixel, upper half}.
    localparam int QD = 16;
    reg [28:0] q_addr [0:QD-1];
    reg [31:0] q_data [0:QD-1];
    reg        q_hi   [0:QD-1];
    reg [3:0]  q_rd = '0, q_wr = '0;
    reg [4:0]  q_cnt = '0;
    wire       q_has = (q_cnt != 0);

    reg  sel = 1'b0;                              // 1: the port carries the queue head
    wire b_cmd   = sel && q_has;
    wire a_cmd   = !sel && (a_rd || a_we);
    wire port_cmd = b_cmd || a_cmd;
    wire accept   = port_cmd && !DDRAM_BUSY;
    wire b_pop    = b_cmd && !DDRAM_BUSY;

    assign DDRAM_ADDR = sel ? q_addr[q_rd] : a_addr;
    assign DDRAM_DIN  = sel ? {2{q_data[q_rd]}} : a_din;
    assign DDRAM_BE   = sel ? (q_hi[q_rd] ? 8'hF0 : 8'h0F) : a_be;
    assign DDRAM_WE   = sel ? q_has : a_we;
    assign DDRAM_RD   = sel ? 1'b0 : a_rd;
    assign a_busy     = sel || DDRAM_BUSY;        // A's command is accepted only when it is on the port

    wire b_push = b_we && (q_cnt != QD);
    wire [4:0] cnt_next = q_cnt + (b_push ? 5'd1 : 5'd0) - (b_pop ? 5'd1 : 5'd0);

    always @(posedge clk) begin
        if (b_push) begin
            q_addr[q_wr] <= b_addr;
            q_data[q_wr] <= b_din[31:0];
            q_hi[q_wr]   <= b_be[4];
            q_wr <= q_wr + 4'd1;
        end
        if (b_we && q_cnt == QD && b_overflows != 16'hFFFF) b_overflows <= b_overflows + 16'd1;
        if (b_pop) q_rd <= q_rd + 4'd1;
        q_cnt <= cnt_next;

        // grant: only between commands (port idle, or its command accepted this cycle)
        if (!port_cmd || accept)
            sel <= (cnt_next != 0) && (!(a_rd || a_we) || accept && !sel || cnt_next >= QD / 2);
    end
endmodule
