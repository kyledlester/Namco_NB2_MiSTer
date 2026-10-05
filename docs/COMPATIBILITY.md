# Compatibility and known differences from MAME

MAME 0.289 is the reference. Everything below differs from MAME on purpose (a resource trade-off) or by
construction, and none of it is reached by either game in the MAME profiles and captures used for the core
(docs/NB2_HARDWARE_REFERENCE.md); each item says what would change if a game did reach it.

| # | Area | MAME | Core | Effect |
| --- | --- | --- | --- | --- |
| 1 | 68EC020 speed | instruction timing from MAME's 68020 tables | TG68K, one step per 24.192 MHz clock (the NB-1 core's pacing, hardware-proven there) | the CPU finishes its work earlier in each frame; MAME's power-on delay loop runs ~2.6x faster, so the game reaches its title ~0.35 s earlier. Gameplay is frame-locked (VBL) and unaffected. |
| 2 | Work RAM `$208000-$240FFF` | all RAM | the game's hot 4 KiB pages (board record) in block RAM, the rest in SDRAM | cold pages are slower to access (a bus cycle waits for SDRAM); contents are identical |
| 3 | RAM `$1C0000-$1CFFFF`, `$280000-$2FFFFF` | RAM | SDRAM-backed, `$1C8000-$1CFFFF` mirrors `$1C0000-$1C7FFF` and `$280000-$2FFFFF` mirrors `$200000-$27FFFF` | neither game reads these (two one-time writes at reset) |
| 4 | C123 tile VRAM | 64 KiB | 40 KiB (`$680000-$689FFF`); the rest reads 0, writes ignored | everything the tilemaps use is backed |
| 5 | C355 sprite RAM | 128 KiB | the windows the C355 reads plus the page-1 list (NB-1 layout); other words read 0 | words a game writes there and reads back would read 0 (not seen) |
| 6 | Sprite bank index 8-15 | reads past an 8-byte array (undefined) | 0 | codes >= `$4000` would differ (not seen) |
| 7 | CHR / ROZ tile number beyond the ROMs | modulo the region's tile count | wraps in the power-of-two window (8 MiB) | only for tile banks past the ROMs (not used) |
| 8 | ROZ colours 8-15 | MAME's shadow pens (`$2000+`) | wrap into the normal pens | colours 0-2 are used |
| 9 | C352 addresses with A22 set | the region mirror (Mach Breakers) or 0 (Outfoxies) | the A22-clear address | Outfoxies never addresses above 2 MiB |
| 10 | C169 wrap-disable bit | ignored by MAME | ignored | as MAME |
