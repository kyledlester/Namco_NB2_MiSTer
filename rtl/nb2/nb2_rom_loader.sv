// Namco NB-2 MiSTer core -- ROM stream loader.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The NB-1 core's nb1_rom_loader (docs/PROVENANCE.md) for the NB-2 map: the ioctl index-0 stream is written
// word by word to its physical place (stream offset == physical address, nb2_mem_pkg); regions below
// 0x2000000 go to the SDRAM front end (client port sd_*), the OBJ region to the DDR3 sprite store (obj_*).
// NB-2 changes: 26-bit offsets, the second client port, and the zero filler between the SDRAM part and the
// DDR3 part of the stream (0x1E84000-0x1FFFFFF) is accepted and dropped without an error.
//
// hps_io WIDE=1 delivers stream bytes 2k and 2k+1 as ioctl_dout[7:0] and [15:8] with ioctl_addr = 2k; storage
// is big-endian, so the value stored is {byte 2k, byte 2k+1}. One word is buffered; ioctl_wait holds hps_io
// until its write has been answered. A word arriving while one is pending is an overrun (sticky error).
module nb2_rom_loader #(parameter [15:0] INDEX = 16'd0) (
    input  wire        clk_sys,
    input  wire        init,

    input  wire        ioctl_download,
    input  wire [15:0] ioctl_index,
    input  wire        ioctl_wr,
    input  wire [26:0] ioctl_addr,
    input  wire [15:0] ioctl_dout,
    output wire        ioctl_wait,

    // SDRAM front-end client (write-only use)
    output wire        sd_valid,
    input  wire        sd_ready,
    input  wire        sd_rsp_valid,
    // DDR3 sprite-store client (write-only use)
    output wire        obj_valid,
    input  wire        obj_ready,
    input  wire        obj_rsp_valid,
    // request fields shared by both ports
    output wire [3:0]  req_region,
    output wire [25:0] req_offset,
    output wire        req_we,
    output wire [31:0] req_wdata,
    output wire [3:0]  req_be,
    input  wire        rsp_err,              // the answering port's error flag (valid with its rsp_valid)

    output reg         loading      = 1'b0,
    output reg         rom_loaded   = 1'b0,
    output reg  [26:0] stream_bytes = '0,    // highest written address + 2
    output reg         err_overrun  = 1'b0,
    output reg         err_range    = 1'b0,
    output reg         err_write    = 1'b0
);
    import nb2_mem_pkg::*;

    wire selected = ioctl_download && (ioctl_index == INDEX);
    wire wr_now   = selected && ioctl_wr;

    reg        pending  = 1'b0;
    reg        inflight = 1'b0;
    reg [26:0] p_addr   = '0;
    reg [15:0] p_data   = '0;

    assign ioctl_wait = pending || wr_now;

    // timing: the buffered word's region is decoded into registers the clock after it arrives (loc_ok)
    loc_t loc_c, loc;
    always_comb loc_c = stream_to_region(p_addr);
    reg  loc_ok = 1'b0;
    reg  filler = 1'b0;
    always @(posedge clk_sys) begin
        loc    <= loc_c;
        filler <= !loc_c.hit && (p_addr >= 27'h1E84000) && (p_addr < 27'h2000000);
        loc_ok <= pending && !(wr_now && !pending);
        if (wr_now || init) loc_ok <= 1'b0;
    end
    wire is_obj = (loc.region == REG_OBJ);

    assign sd_valid   = pending && loc_ok && !inflight && loc.hit && !is_obj;
    assign obj_valid  = pending && loc_ok && !inflight && loc.hit && is_obj;
    assign req_region = loc.region;
    assign req_offset = {loc.offset[25:2], 2'b00};
    assign req_we     = 1'b1;
    assign req_be     = p_addr[1] ? 4'b0011 : 4'b1100;
    assign req_wdata  = p_addr[1] ? {16'h0000, p_data[7:0], p_data[15:8]}
                                  : {p_data[7:0], p_data[15:8], 16'h0000};
    wire rsp_valid = inflight && (sd_rsp_valid || obj_rsp_valid);

    reg selected_q = 1'b0;

    always @(posedge clk_sys) begin
        if (init) begin
            pending <= 1'b0; inflight <= 1'b0; selected_q <= 1'b0;
            loading <= 1'b0; rom_loaded <= 1'b0; stream_bytes <= '0;
            err_overrun <= 1'b0; err_range <= 1'b0; err_write <= 1'b0;
        end else begin
            selected_q <= selected;
            if (selected && !selected_q) begin
                loading      <= 1'b1;
                rom_loaded   <= 1'b0;
                stream_bytes <= '0;
                err_overrun  <= 1'b0;
                err_range    <= 1'b0;
                err_write    <= 1'b0;
            end
            if (wr_now) begin
                if (pending) err_overrun <= 1'b1;
                else begin
                    pending <= 1'b1;
                    p_addr  <= ioctl_addr;
                    p_data  <= ioctl_dout;
                    if (ioctl_addr + 27'd2 > stream_bytes) stream_bytes <= ioctl_addr + 27'd2;
                end
            end
            if (pending && loc_ok && !inflight && !loc.hit) begin
                if (!filler) err_range <= 1'b1;       // outside the map: drop (the filler silently)
                pending <= 1'b0;
            end
            if ((sd_valid && sd_ready) || (obj_valid && obj_ready)) inflight <= 1'b1;
            if (rsp_valid) begin
                if (rsp_err) err_write <= 1'b1;
                inflight <= 1'b0;
                pending  <= 1'b0;
            end
            if (loading && !selected && !pending) begin
                loading    <= 1'b0;
                rom_loaded <= 1'b1;
            end
        end
    end
endmodule
