# Building the core

## Requirements

* Intel Quartus Prime Lite **17.0** (17.0.2 recommended, as for other MiSTer cores) with Cyclone V device
  support.
* About 30-45 minutes and 8 GB of RAM per full compile.

## Build

Open `Namco_NB2.qpf` in Quartus and run **Processing > Start Compilation**, or from PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/build.ps1
```

Pass `-QuartusRoot <path>` if Quartus is not installed in `C:\intelFPGA_lite\17.0\quartus`.

Outputs:

* `output_files/Namco_NB2.rbf`, the bitstream.
* `Releases/Namco_NB2_YYYYMMDD.rbf`, a dated copy made by the post-flow script `scripts/release_rbf.tcl` after
  every full compile.

Check the timing report (`output_files/Namco_NB2.sta.summary`): all clock domains should have non-negative setup
and hold slack. The released build uses fitter seed 5 (`Namco_NB2.qsf`) and closes with +0.140 ns setup and
+0.210 ns hold slack. Block RAM is nearly full (546 of 553 M10K blocks).

## MRA

The MRAs are generated, not hand-written, one JSON per MAME set in `scripts/mra/games/` (shown for `outfxies`;
every other set works the same way, with clone MRAs in `MRA/_alternatives/_<Game>/`):

```powershell
python scripts/mra/nb2_mra.py generate --game scripts/mra/games/outfxies.json -o "MRA/The Outfoxies (World, OU2).mra"
python scripts/mra/nb2_mra.py validate --mra "MRA/The Outfoxies (World, OU2).mra" --game scripts/mra/games/outfxies.json
```

`validate` rebuilds the ROM stream the way MiSTer's MRA loader does and checks it against MAME's region layout.
With `--zip "outfxies.zip|namcoc75.zip"` it also checks every CRC against your own ROM set (nothing derived from
the ROMs is written). For a clone, list the clone's zip first, then the parent's and `namcoc75.zip`:
`--zip "outfxiesj.zip|outfxies.zip|namcoc75.zip"`. With `--mame-regions <dir>` it also compares the whole stream
byte for byte with the regions MAME assembled, dumped by `scripts/mame/nb2_regions.lua` (keep that output outside
the repository). See [MRA_FORMAT.md](MRA_FORMAT.md).

## Benches

`sim/` holds three self-contained benches:

* `nb2_mem_tb.sv`: the NB-2 SDRAM controller and client scheduler on a strict SDRAM model (`sim/models/`),
  random traffic from every client checked against a shadow memory, plus a throughput measurement.
* `nb2_ddr_mux_tb.sv`: the DDR3 port shared by the sprite store and the framework's `screen_rotate`, on a DDR3
  model with random busy cycles and latency (Verilator: `bash scripts/sim/vl_build_ddr_mux.sh`).
* `m22_c75_alu_tb.sv`: the exhaustive equivalence check for the timing changes in the M37702 core
  (`rtl/vendor/README.md`), inherited from the NB-1 core.

The MAME-comparison benches used during development (system, video and audio comparisons) need locally captured
MAME data and are not part of this repository.

## Install on MiSTer

1. Copy `Releases/Namco_NB2_YYYYMMDD.rbf` to `_Arcade/cores/`.
2. Copy the `MRA/*.mra` files to `_Arcade/`, and the `MRA/_alternatives/_<Game>` folders to
   `_Arcade/_alternatives/`.
3. Put the game zips in `games/mame/`, plus `namcoc75.zip` if you use split or merged sets.

See [COMPATIBILITY.md](COMPATIBILITY.md) for the known differences from MAME.
