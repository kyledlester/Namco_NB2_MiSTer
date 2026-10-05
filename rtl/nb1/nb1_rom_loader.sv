// Namco NB-1 MiSTer core -- ROM stream loader (M2).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Writes the MRA ioctl stream (hps_io WIDE=1) into logical storage through
// one nb1_memory client port. Game-independent: the stream layout is the
// fixed platform map in nb1_mem_pkg (stream offset == SDRAM byte address).
//
// hps_io WIDE=1 delivers stream bytes 2k and 2k+1 as ioctl_dout[7:0] and
// [15:8] with ioctl_addr = 2k. Storage is big-endian (nb1_mem_pkg), so the
// 16-bit value stored is {byte 2k, byte 2k+1} = {dout[7:0], dout[15:8]}.
// This one swap is the only data transformation in the loader; the MRA
// already delivers MAME's region bytes in MAME's order.
//
// Flow control: one word is buffered. ioctl_wait is asserted combinationally
// in the cycle of ioctl_wr and stays high until the word's memory response
// arrives, so hps_io never presents a second word while one is pending. A
// word arriving while one is pending anyway is counted as an overrun (sticky
// error, never silently written).
//
// Words at stream addresses outside every region are dropped and flagged
// (err_range); they can never alias into another region.
//
// State: rom_loaded/stream_bytes/errors survive OSD and user resets. They
// are cleared only when a new index-0 download starts, or by `init` (PLL
// loss of lock, which also re-initialises the SDRAM contents' controller).
module nb1_rom_loader #(parameter [15:0] INDEX = 16'd0) (
    input  wire        clk_sys,
    input  wire        init,

    input  wire        ioctl_download,
    input  wire [15:0] ioctl_index,
    input  wire        ioctl_wr,
    input  wire [26:0] ioctl_addr,
    input  wire [15:0] ioctl_dout,
    output wire        ioctl_wait,

    // nb1_memory client port (write-only use)
    output wire        req_valid,
    input  wire        req_ready,
    output wire [3:0]  req_region,
    output wire [24:0] req_offset,
    output wire        req_we,
    output wire [31:0] req_wdata,
    output wire [3:0]  req_be,
    input  wire        rsp_valid,
    input  wire        rsp_err,

    output reg         loading      = 1'b0,
    output reg         rom_loaded   = 1'b0,
    output reg  [26:0] stream_bytes = '0,    // highest written address + 2
    output reg         err_overrun  = 1'b0,
    output reg         err_range    = 1'b0,
    output reg         err_write    = 1'b0    // memory answered rsp_err to a mapped write
);
    import nb1_mem_pkg::*;

    wire selected = ioctl_download && (ioctl_index == INDEX);
    wire wr_now   = selected && ioctl_wr;

    reg        pending  = 1'b0;   // a word is buffered
    reg        inflight = 1'b0;   // it has been accepted by memory
    reg [26:0] p_addr   = '0;
    reg [15:0] p_data   = '0;

    assign ioctl_wait = pending || wr_now;

    loc_t loc;
    always_comb loc = stream_to_region(p_addr);

    assign req_valid  = pending && !inflight && loc.hit;
    assign req_region = loc.region;
    assign req_offset = {loc.offset[24:2], 2'b00};
    assign req_we     = 1'b1;
    assign req_be     = p_addr[1] ? 4'b0011 : 4'b1100;
    assign req_wdata  = p_addr[1] ? {16'h0000, p_data[7:0], p_data[15:8]}
                                  : {p_data[7:0], p_data[15:8], 16'h0000};

    reg selected_q = 1'b0;

    always @(posedge clk_sys) begin
        if (init) begin
            pending <= 1'b0; inflight <= 1'b0; selected_q <= 1'b0;
            loading <= 1'b0; rom_loaded <= 1'b0; stream_bytes <= '0;
            err_overrun <= 1'b0; err_range <= 1'b0; err_write <= 1'b0;
        end else begin
            selected_q <= selected;

            if (selected && !selected_q) begin        // new download: forget the old image
                loading      <= 1'b1;
                rom_loaded   <= 1'b0;
                stream_bytes <= '0;
                err_overrun  <= 1'b0;
                err_range    <= 1'b0;
                err_write    <= 1'b0;
            end

            if (wr_now) begin
                if (pending) begin
                    err_overrun <= 1'b1;
                end else begin
                    pending <= 1'b1;
                    p_addr  <= ioctl_addr;
                    p_data  <= ioctl_dout;
                    if (ioctl_addr + 27'd2 > stream_bytes) stream_bytes <= ioctl_addr + 27'd2;
                end
            end

            if (pending && !inflight && !loc.hit) begin
                err_range <= 1'b1;                    // outside the map: drop
                pending   <= 1'b0;
            end
            if (req_valid && req_ready) inflight <= 1'b1;
            if (inflight && rsp_valid) begin
                if (rsp_err) err_write <= 1'b1;
                inflight <= 1'b0;
                pending  <= 1'b0;
            end

            // Download finished and the last word committed.
            if (loading && !selected && !pending) begin
                loading    <= 1'b0;
                rom_loaded <= 1'b1;
            end
        end
    end
endmodule
