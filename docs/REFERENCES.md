# Credits and third-party components

## Included code

| Component | Path | Origin | License |
| --- | --- | --- | --- |
| TG68K.C 68020 core | `rtl/vendor/tg68k/` | [TobiFlex/TG68K.C](https://github.com/TobiFlex/TG68K.C) by Tobias Gubener and contributors, commit `ade33e3`, unmodified (see `rtl/vendor/tg68k/README.md`) | LGPL-3.0-or-later |
| M37702 CPU core and M37702M2 peripherals (C75) | `rtl/vendor/m37702/` | The author's [Namco_NA1_NA2_MiSTer](https://github.com/kyledlester/Namco_NA1_NA2_MiSTer) core, with documented local modifications (see `rtl/vendor/README.md`) | GPL-3.0-or-later |
| CRT Adjust | `rtl/vendor/crt_adjust.sv` | MiSTer-CRT-Adjust by Umberto Parisi (rmonic79) with Andrea Bogazzi, unmodified | GPL-3.0-or-later |
| SDRAM controller (NB-1, no longer instantiated) | `rtl/vendor/sdram.sv` | [MiSTer-devel/GBA_MiSTer](https://github.com/MiSTer-devel/GBA_MiSTer) via the NA-1/NA-2 and NB-1 cores (see `rtl/vendor/README.md`); the NB-2 controller `rtl/nb2/nb2_sdram.sv` keeps its physical interface | GPL-3.0-or-later |
| MiSTer framework | `sys/` | [MiSTer-devel/Template_MiSTer](https://github.com/MiSTer-devel/Template_MiSTer), including `screen_rotate` for the Vertical CW/CCW orientations; one local timing change in `sys/yc_out.sv` (bit-identical output) | see `LICENSE.MiSTer` and file headers |
| NB-1 core | `rtl/nb1/`, much of `rtl/nb2/` | The author's [Namco_NB1_MiSTer](https://github.com/kyledlester/Namco_NB1_MiSTer) core (see [PROVENANCE.md](PROVENANCE.md)) | GPL-3.0-or-later |

## Behavioural references

The core's logic was written for this project. Its behaviour follows these MAME 0.289 sources
(BSD-3-Clause):

* [`src/mame/namco/namconb1.cpp`](https://github.com/mamedev/mame/blob/mame0289/src/mame/namco/namconb1.cpp):
  board, NB-2 memory map, interrupts, KEYCUS, per-game graphics bank callbacks and ROM definitions
* `src/mame/namco/namco_c169roz.cpp`: the C169 rotate/zoom layers
* `src/mame/namco/namco_c123tmap.cpp`, `src/mame/shared/namco_c355spr.cpp`, `src/mame/namco/namco_c116.cpp`:
  tilemaps, sprites, palette
* `src/devices/cpu/m37710/` and `src/mame/namco/namcomcu.cpp`: the C75 (M37702) instruction set, timing and
  peripherals
* `src/devices/sound/c352.cpp`: C352 PCM

Other references: the author's NB-1 and NA-1/NA-2 cores (C75 core, sound-chip sequencer structure, video and
memory front end), and MiSTer's `Main_MiSTer` MRA loader (`support/arcade/mra_loader.cpp`), which the MRA tool
emulates to validate every MRA.

## Game data

No ROMs, MCU BIOS images or other game data are included. The MRAs list MAME part names and CRCs only.
