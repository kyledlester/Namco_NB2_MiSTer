// Namco NB-1 MiSTer core -- EEPROM persistence through MiSTer NVRAM (M18).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The persistence layer around the CPU-visible 2816 (nb1_eeprom). It owns
// the EEPROM storage's second port and never touches the CPU port. Platform
// level: nothing here knows which NB-1 game is loaded (every NB-1/NB-2 set
// has one 2 KiB 28C16, MAME EEPROM_2816; docs/M18_RESEARCH.md).
//
// MiSTer mechanism (Main_MiSTer mra_loader.cpp / menu.cpp, hps_io.sv):
// - Load: the MRA's <nvram index="INDEX" size="2048"/> makes the HPS send
//   the saved .nvm file (if one exists) as an ioctl download on INDEX while
//   the MRA loads. The MRA sends an erased image on the same index just
//   before it (<rom index="INDEX">, $FF x 2048; a game whose MAME set has an
//   "eeprom" default region would send that region instead), so the
//   contents are defined whether or not a save exists.
// - Save: when the OSD main menu is entered, the HPS asks the core
//   (UIO_CHK_UPLOAD). hps_io answers yes once per rising edge of
//   ioctl_upload_req; the HPS then reads the whole image as an ioctl upload
//   on INDEX and writes the .nvm file.
//
// File format = MAME's nvram/<set>/eeprom: cell 0 first. With hps_io WIDE=1
// a word carries file bytes {2k+1, 2k} = {odd cell, even cell}; the storage
// port is in the CPU word view {even, odd}, so bytes are swapped both ways.
//
// Dirty state (the NA-1/NA-2 M22/M27 lessons, docs/M18_RESEARCH.md s.5):
// - `dirty` is set only by a committed CPU write (`cpu_write`, nb1_eeprom).
//   Downloads write through the persistence port and never set it.
// - It is cleared when a session on INDEX starts: an upload (the save that
//   consumes it) or a download (a restore replaces the contents). A CPU
//   write during an upload sets it again, so that data is saved next time.
// - `upload_req` = `dirty`: hps_io latches its rising edge, so one save is
//   requested per clean -> dirty transition, never periodically.
// Download writes are single-cycle on the private port (no wait). Upload
// reads hold ioctl_wait for the two clocks after each ioctl_rd so that
// ioctl_din always belongs to the current ioctl_addr.
module nb1_nvram #(
    parameter [15:0] INDEX = 16'd1,
    parameter int    BYTES = 2048
) (
    input  wire        clk_sys,
    // hps_io ioctl (WIDE = 1)
    input  wire        ioctl_download,
    input  wire        ioctl_upload,
    input  wire [15:0] ioctl_index,
    input  wire        ioctl_wr,
    input  wire        ioctl_rd,
    input  wire [26:0] ioctl_addr,
    input  wire [15:0] ioctl_dout,
    output wire [15:0] ioctl_din,
    output wire        ioctl_wait,
    output wire        upload_req,
    // nb1_eeprom persistence port (word view: D15:8 = even byte)
    output wire        mem_en,
    output wire        mem_we,
    output wire [10:1] mem_addr,
    output wire [15:0] mem_wdata,
    input  wire [15:0] mem_rdata,
    input  wire        cpu_write,
    // diagnostics (overlay row 17)
    output reg         dirty = 1'b0,
    output reg  [11:0] dirty_events = '0,  // CPU EEPROM writes (saturating)
    output reg  [3:0]  loads = '0,         // download sessions on INDEX (saturating)
    output reg  [7:0]  saves = '0,         // upload sessions on INDEX (saturating)
    output reg  [11:0] last_bytes = '0,    // bytes moved by the last session
    output reg  [1:0]  last_action = '0    // 0 none, 1 load, 2 save
);
    wire sel_dl = ioctl_download && (ioctl_index == INDEX);
    wire sel_ul = ioctl_upload   && (ioctl_index == INDEX);
    wire in_range = (ioctl_addr < BYTES);

    assign mem_en    = (sel_dl && ioctl_wr && in_range) || sel_ul;
    assign mem_we    = sel_dl && ioctl_wr && in_range;
    assign mem_addr  = ioctl_addr[10:1];
    assign mem_wdata = {ioctl_dout[7:0], ioctl_dout[15:8]};
    assign ioctl_din = {mem_rdata[7:0], mem_rdata[15:8]};

    reg rd_d = 1'b0;
    always @(posedge clk_sys) rd_d <= sel_ul && ioctl_rd;
    assign ioctl_wait = sel_ul && (ioctl_rd || rd_d);

    assign upload_req = dirty;

    // The CPU write pulse is decoded from the CPU's bus-cycle address (the R-TIMING-1
    // cone); it is registered once here so no CPU path reaches the counters (M18 build 2
    // showed kcap_addr -> dirty_events at -0.06 ns). Dirty rises one clock later.
    reg cpu_write_q = 1'b0;
    reg sel_dl_d = 1'b0, sel_ul_d = 1'b0;
    always @(posedge clk_sys) begin
        cpu_write_q <= cpu_write;
        sel_dl_d <= sel_dl;
        sel_ul_d <= sel_ul;
        if (cpu_write_q) begin
            dirty <= 1'b1;
            if (dirty_events != 12'hFFF) dirty_events <= dirty_events + 12'd1;
        end else if ((sel_dl && !sel_dl_d) || (sel_ul && !sel_ul_d)) begin
            dirty <= 1'b0;
        end
        if (sel_dl && !sel_dl_d && loads != 4'hF)  loads <= loads + 4'd1;
        if (sel_ul && !sel_ul_d && saves != 8'hFF) saves <= saves + 8'd1;
        // Session end: hps_io has already advanced ioctl_addr past the last word.
        if (!sel_dl && sel_dl_d) begin last_bytes <= ioctl_addr[11:0]; last_action <= 2'd1; end
        if (!sel_ul && sel_ul_d) begin last_bytes <= ioctl_addr[11:0]; last_action <= 2'd2; end
    end
endmodule
