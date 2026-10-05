# Namco NB-2 for MiSTer

A MiSTer FPGA core for Namco's **NB-2** arcade board (1994-1995): **The Outfoxies** and **Mach Breakers**.
One core (`Namco_NB2`) runs both games; each game has its own MRA.

I created this core because I wanted to play these games on my MiSTer FPGA. I am posting it here and open sourcing it for everyone to enjoy and give feedback/make improvements. This core was created with the assistance of AI tooling.

The Outfoxies is a two-player arena shooter/fighter with zooming, rotating stages; Mach Breakers is a
four-player button-mashing sports game, the sequel to Numan Athletics.

**Status: beta.** Both games boot and are playable on real MiSTer hardware.

## Quick start

1. Copy the core from [`Releases/`](Releases/) (`Namco_NB2_YYYYMMDD.rbf`) to
   **`/media/fat/_Arcade/cores/`**.
2. Copy the MRA files from [`MRA/`](MRA/) to **`/media/fat/_Arcade/`**.
   For region/revision variants, copy the `_<Game>` folders from
   [`MRA/_alternatives/`](MRA/_alternatives/) to **`/media/fat/_Arcade/_alternatives/`**.
3. Put the MAME ROM zips (MAME 0.289 sets) in **`/media/fat/games/mame/`**.
   With split or merged sets, also add the C75 MCU BIOS zip **`namcoc75.zip`** there.
   Non-merged sets already include the BIOS in each game's zip.
4. Load a game from the **Arcade** menu.

ROMs are not included. You must supply your own.

Any SDRAM module size works (32 MB is enough): the sprite graphics are kept in the DE10-Nano's own DDR3 memory.
After loading, the screen stays black for about 3 seconds while the core checks the ROMs it received.

## Supported games

| Game | MAME set | Year | Genre | Players | Status |
| --- | --- | --- | --- | --- | --- |
| The Outfoxies (World, OU2) | `outfxies` | 1994 | Action / fighting | 1-2 | Boots to title screen and is playable |
| Mach Breakers (World, MB2) | `machbrkr` | 1995 | Sports | 1-4 | Boots to title screen and is playable |

### Alternatives

These use the same core with a different ROM set. Each one also needs its parent's zip.

| Game | MAME set | Year | Parent game | Status |
| --- | --- | --- | --- | --- |
| The Outfoxies (Japan, OU1) | `outfxiesj` | 1994 | The Outfoxies | Boots to title screen and is playable |
| The Outfoxies (Japan, OU1, alternate GFX ROMs) | `outfxiesja` | 1994 | The Outfoxies | Boots to title screen and is playable |
| The Outfoxies (Korea?) | `outfxiesa` | 1994 | The Outfoxies | Not yet tested |
| Mach Breakers - Numan Athletics 2 (Japan, MB1) | `machbrkrj` | 1995 | Mach Breakers | Boots to title screen and is playable |

### Controls

Both games use an 8-way joystick, three buttons, Start and Coin per player, plus Service. Map them in MiSTer's
controller setup (**Button 1-3, Start, Coin, Service**).

* **The Outfoxies**: joystick and Buttons 1-3.
* **Mach Breakers** uses **only Buttons 1, 2 and 3** (and Start/Coin); it doesn't use the joystick directions.
  MiSTer still asks you to map the d-pad when you set up the controls, because that is required for every core,
  but the directions do nothing in this game.

### ROM notes

* Split or merged sets need **`namcoc75.zip`** (the C75 MCU BIOS) in `games/mame/`.
* Each alternative needs its own zip **and** its parent's (`outfxies.zip` or `machbrkr.zip`).

More detail: [docs/COMPATIBILITY.md](docs/COMPATIBILITY.md).

## About the hardware

NB-2 is Namco's mid-1990s 32-bit board:

* Motorola **68EC020** main CPU.
* Namco **C75** MCU, a Mitsubishi M37702 running Namco's BIOS. It handles the controls and drives the **C352**
  32-voice PCM sound chip.
* **C123** tilemaps, two **C169** rotate/zoom layers (with a per-line mode), **C355** sprites with per-game
  graphics banking, and the **C116** palette and display window.
* A per-game **KEYCUS** protection chip (C390 on The Outfoxies).
* Game settings and high scores kept in an EEPROM.

## About the core

* Runs the genuine C75 MCU BIOS on an M37702 CPU core, so sound and inputs come from Namco's own firmware.
* One RBF for both games. The core has no per-game code; each MRA carries a small board record (protection chip,
  graphics bank transforms, which work-RAM pages to keep in block RAM). See [docs/MRA_FORMAT.md](docs/MRA_FORMAT.md).
* Full video: all tilemap layers, both rotate/zoom layers including the per-line mode, sprites, priorities and the
  display window. In simulation the picture matches MAME pixel for pixel through attract mode and gameplay of both
  games.
* Sprite graphics (20 MB) in the DE10-Nano's DDR3, everything else in SDRAM, so any SDRAM module works.
* Native 15 kHz output for CRTs, with optional CRT Adjust (H-size, H-position, V-shift) thanks to
  rmonic79/MiSTer-CRT-Adjust.
* OSD options: aspect ratio, orientation (Horizontal, Vertical CCW, Vertical CW, Flipped), scandoubler effects,
  CRT Adjust, Service Mode, stereo mix, Reset.
  * **Vertical CCW / CW** turn the HDMI picture 90 degrees for a monitor on its side; the analog output stays
    horizontal.
  * **Flipped** turns the picture 180 degrees on both HDMI and the analog/15 kHz output, for an upside-down
    monitor.
* EEPROM saved to the SD card as `.nvm`. MiSTer writes it when you open the OSD after the game has changed its
  settings or scores.
* Up to four players (Mach Breakers).

MAME's `namconb1` driver (0.289) is the behavioural reference.

### Known issues

* The core's 68EC020 runs the games' power-on delay loop faster than MAME, so each game reaches its title screen
  about a third of a second sooner. Gameplay is locked to the video frame and is unaffected.
* See [docs/COMPATIBILITY.md](docs/COMPATIBILITY.md) for the other (deliberate) differences from MAME.

## Releases

Builds are in [`Releases/`](Releases/) as `Namco_NB2_YYYYMMDD.rbf`. The MRAs name the core without the date
(`<rbf>Namco_NB2</rbf>`), and MiSTer loads the newest dated file in `_Arcade/cores/`. You are welcome to run your
own build if you'd prefer.

## Building

Quartus Prime Lite 17.0. Open `Namco_NB2.qpf` and compile; the build copies a dated RBF into `Releases/`. See
[docs/BUILDING.md](docs/BUILDING.md).

## Documentation

* [Compatibility](docs/COMPATIBILITY.md)
* [MRA format](docs/MRA_FORMAT.md)
* [Architecture](docs/ARCHITECTURE.md)
* [NB-2 hardware reference](docs/NB2_HARDWARE_REFERENCE.md)
* [Building](docs/BUILDING.md)
* [Provenance](docs/PROVENANCE.md)
* [Credits and third-party components](docs/REFERENCES.md)

## Credits

* MAME driver `namconb1.cpp` (Phil Stroffolino) and the devices it uses: C169 rotate/zoom and C123 tilemaps
  (David Haywood, Phil Stroffolino), C355 sprites (David Haywood, Phil Stroffolino, Bryan McPhail), C116 palette
  and Namco MCU (Alex W. Jackson), C352 sound (R. Belmont, superctr), M37710 CPU (R. Belmont, Karl Stenerud) —
  the behavioural reference for this core (no MAME code is included).
* TG68K.C 68020 core — Tobias Gubener and contributors.
* M37702 CPU core for the C75 MCU — from the author's
  [Namco NA-1/NA-2 core](https://github.com/kyledlester/Namco_NA1_NA2_MiSTer).
* MiSTer framework — Sorgelig (Alexey Melnikov) and contributors, including `screen_rotate`, the scandoubler (with
  Till Harbaum) and HQ2x (Ludvig Strigeus); the SDRAM controller keeps the physical interface of Sorgelig's MiSTer
  `sdram.sv`.
* CRT Adjust — Umberto Parisi ([rmonic79/MiSTer-CRT-Adjust](https://github.com/rmonic79/MiSTer-CRT-Adjust)), with
  Andrea Bogazzi (asturur).

Full list with revisions and licences: [docs/REFERENCES.md](docs/REFERENCES.md).

## License

GPL-3.0-or-later (see [LICENSE](LICENSE)). The MiSTer framework in `sys/` keeps its own notices
([LICENSE.MiSTer](LICENSE.MiSTer)); TG68K.C is LGPL-3.0-or-later; the M37702 core and CRT Adjust are
GPL-3.0-or-later. See [docs/REFERENCES.md](docs/REFERENCES.md).

No ROMs, MCU BIOS images or other game data are included in this repository.
