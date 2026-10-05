# NB-2 core architecture

One RBF (`Namco_NB2`) for both NB-2 games; everything game-specific (KEYCUS, bank transforms, work-RAM page map)
arrives in the MRA board record, as on NB-1. Evidence for the board: [NB2_HARDWARE_REFERENCE.md](NB2_HARDWARE_REFERENCE.md).

## 1. What is reused from NB-1 (unchanged unless noted)

68EC020 wrapper and credit scheduler (`nb1_cpu`), program cache, the C75 (M37702 + BIOS + glue), shared RAM and
its lock, C352, C116 palette, C123 control/VRAM/renderer (tile callback added), C355 RAM/renderer (bank callback
added), EEPROM + NVRAM, inputs, IRQ logic (register placement changed), clock enables, raster, reset, ROM loader and
checker, CRT Adjust, framework. Each changed module is copied to `rtl/nb2/` (see [PROVENANCE.md](PROVENANCE.md)).

## 2. Memory decision (2026-10-03)

### 2.1 Problem

| | Outfoxies | Mach Breakers |
| --- | --- | --- |
| ROM data | 36.5 MiB | 37.5 MiB |
| of which sprites (OBJ) | 20 MiB | 20 MiB |

The ROM data does not fit the 32 MiB that every MiSTer SDRAM module provides. Bandwidth is the harder limit: both
games draw their gameplay background with the two C169 ROZ layers, which fetch a pixel byte and a mask bit at an
arbitrary graphics address for every screen pixel of each layer. Outfoxies zooms out up to 4 source pixels per
screen pixel, so a 64-bit SDRAM read serves only 2 pixels: about 290 pixel reads + 140 mask reads per line for
the two layers, plus ~220 tile reads and the CPU, against ~700 random reads per 63.5 us line from one SDRAM.
The NB-1 core already needed all of its SDRAM time for tiles and sprites.

### 2.2 Decision

* **Sprite graphics (OBJ, 20 MiB) live in DDR3** (the core's own f2sdram port, `DDRAM_*`), at byte `0x32000000`.
  The C355 renderer works three lines ahead and fetches 16-byte sprite rows whose addresses are known early, so it
  tolerates DDR3 latency; its bandwidth (well under 200 reads per line) is small for DDR3.
* The OSD Orientation Vertical CW/CCW uses the framework's `screen_rotate` framebuffer (`MISTER_FB`, DDR3 at
  `0x24000000`, no overlap). Its writes (one per output pixel, never waiting) share the DDR3 port through
  `rtl/nb2/nb2_ddr_mux.sv`: a 16-entry queue written into the gaps between sprite-store commands, first once half
  full. Flipped (180 degrees) needs no memory: the renderers draw the lines bottom-up and the output reads x
  mirrored (`nb2_core` `osd_flip`).
* **Everything else lives in the 32 MiB SDRAM** (works with every SDRAM module): ROZ pixels and masks, C123 CHR and
  SHAPE, program, data ROM, C75 data, C352 voices, and the cold work-RAM pages. SDRAM time goes first to the ROZ and
  tile renderers.
* **Block RAM**: ROZ VRAM (128 KiB) is new and must be on-chip (random per-pixel lookups). It is paid for by moving the
  rarely used work-RAM pages to SDRAM: each game's board record lists its hot 4 KiB pages (MAME profile, reference
  2.1), which stay in block RAM; the other pages of `$208000-$240FFF` are backed by SDRAM (correct, only slower).
  The C123 VRAM is reduced to the 40 KiB both games use (`$680000-$689FFF`); the rest reads `$FFFF` as on NB-1's
  partial maps, counted.

### 2.3 ROM stream (MRA index 0) and placement

Stream offset = physical address, as on NB-1. Offsets below `0x2000000` go to SDRAM, the rest to DDR3.

| Region | MAME region | Window | Stream / SDRAM base |
| --- | --- | --- | --- |
| ROZ | `c169roz` | 8 MiB | `0x0000000` |
| CHR | `c123tmap` | 8 MiB | `0x0800000` |
| VOICE | `c352` (address bit 22 dropped: phys = {A23, A21..A0}) | 8 MiB | `0x1000000` |
| SHAPE | `c123tmap:mask` | 2 MiB | `0x1800000` |
| ROZMASK | `c169roz:mask` | 2 MiB | `0x1A00000` |
| PROG | `maincpu` | 1 MiB | `0x1C00000` |
| DATA | `data` | 1 MiB | `0x1D00000` |
| C75DATA | `c75data` | 512 KiB | `0x1E00000` |
| C75BIOS | `mcu:internal` | 16 KiB | `0x1E80000` |
| (WRAM backing, not streamed) | | 512 KiB | `0x1F00000` |
| OBJ | `c355spr` (first 20 MiB; reads beyond return 0 as MAME's zero fill) | 20 MiB | stream `0x2000000` -> DDR3 |

### 2.4 Risks and how they are checked

* DDR3 latency spikes (HPS activity) could drop sprite lines. Measured on hardware only; the renderer counts drops.
  Simulation uses a DDR3 model with long random latency.
* SDRAM bandwidth for the worst ROZ zoom: checked in simulation on captured worst-case scenes (zoomed-out Outfoxies,
  Mach Breakers per-line floor) with the real controller model; the ROZ renderer keeps a tile-mask cache and skips
  pixel reads for transparent spans.
