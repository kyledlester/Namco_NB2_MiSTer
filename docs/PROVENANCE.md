# Provenance

This core starts from the author's released Namco NB-1 MiSTer core
([kyledlester/Namco_NB1_MiSTer](https://github.com/kyledlester/Namco_NB1_MiSTer), commit `46437bf`,
"Release 20261002"). NB-1 and NB-2 share most of their Namco custom chips, so much of that core is reused here.
It is a separate core: it plays only the NB-2 games listed in the README.

Taken over from the NB-1 core: `sys/` (MiSTer framework), `rtl/nb1/` (NB-1 modules), `rtl/vendor/` (TG68K.C,
M37702, SDRAM controller, CRT Adjust; see `rtl/vendor/README.md`), `rtl/pll.qip`, `scripts/build.ps1`,
`scripts/release_rbf.tcl`, the MRA tool (now `scripts/mra/nb2_mra.py`), `LICENSE` and `LICENSE.MiSTer`.

How the code is organised:

* `rtl/nb1/` holds the NB-1 modules the NB-2 core uses as they are (CPU wrapper, C75 sound MCU, EEPROM, KEYCUS,
  interrupts, palette, video timing and similar). A module that needed an NB-2 change was copied to `rtl/nb2/`
  under an `nb2_` name, with the change described in its header, so every difference from the NB-1 core shows up
  in a plain diff. One exception: `rtl/nb1/nb1_share_lock.sv` carries a small NB-2 timing change in place
  (described in its header).
* NB-2-only hardware lives in `rtl/nb2/`: the C169 ROZ renderer, the NB-2 address map, the per-game graphics bank
  transforms, the pipelined SDRAM controller, the DDR3 sprite-graphics store and the DDR3 port arbiter.
* Some NB-1 files in `rtl/nb1/` (for example the NB-1 renderers, memory front end and light gun) and
  `rtl/vendor/sdram.sv` are still listed in the project but are not instantiated by the NB-2 core; Quartus ignores
  them.
* No ROM, BIOS, extracted region, trace or generated memory image is part of this repository.

Behavioural reference: MAME 0.289 (tag `mame0289`). Credits and licenses: [REFERENCES.md](REFERENCES.md).
