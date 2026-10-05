# Namco NB-2 hardware reference (milestone 1)

Board reference for the NB-2 core: The Outfoxies (`outfxies`, World OU2) and Mach Breakers (`machbrkr`,
World MB2). The behavioural specification is MAME 0.289 (tag `mame0289`, `d0b7160e`):
`src/mame/namco/namconb1.cpp` (`namconb2_state`), `namco_c169roz.cpp`, `namco_c123tmap.cpp`,
`shared/namco_c355spr.cpp`, `namco_c116.cpp`, `namcomcu.cpp`, `devices/sound/c352.cpp`. Line numbers below are
in the 0.289 files.

Evidence tags:

* **[SRC]** MAME 0.289 source.
* **[OBS]** observed in MAME 0.289 with the Lua captures in `scripts/mame/` (`nb2_profile.lua`: 9,000-frame address
  profile with autoplay; `nb2_boot.lua`: 1,500-frame cold-boot device log; `nb2_snaps.lua`: snapshots + video
  registers every 300 frames). The captures stay on the development machine (`NB2_research\cap`).
* **[PCB]** PCB documentation quoted in the MAME driver header (lines 196-272).
* **[INF]** inferred; the reasoning is given.
* **[NB1]** established for NB-1 by the released NB-1 core and its MAME comparisons; NB-2 uses the same part.

ROM sets: `mame -verifyroms outfxies machbrkr` reports both sets good with `namcoc75.zip` present
(2026-10-03, MAME 0.289).

## 1. Parts and clocks

| Function | NB-2 part | Same as NB-1? |
| --- | --- | --- |
| Main CPU | MC68EC020FG25 at 48.384 MHz / 2 = 24.192 MHz [SRC 1326, PCB] | yes |
| CPU glue | C383, C385 [PCB] (NB-1: C329, 137) | different chips, behaviour modelled by MAME's map only |
| Sound/IO MCU | C75 (M37702, internal BIOS `c75.bin`) at 48.384 / 3 = 16.128 MHz [SRC 1329] | yes |
| PCM | C352 at 24.192 MHz, divider 288 -> 84 kHz, stereo front L/R [SRC 1375-1378, PCB "Audio out is Stereo"] | yes |
| I/O | 160 [PCB] | yes |
| Tilemaps | 123 + 145 + 156 (C123 model) [PCB, SRC] | yes, with game-specific tile callbacks |
| Sprites | C355 + 187 [PCB] | yes, with game-specific bank callbacks |
| ROZ | **C169** + 3 x 384 [PCB] | **NB-2 only** |
| Palette / clip / raster IRQ | C116 [SRC] | yes; clip X offset differs (section 6) |
| Key custom | Outfoxies C390 [PCB]; Mach Breakers none in MAME [SRC 998] | NB-1 uses other C3xx parts |

Video timing [SRC 1351]: 48.384 MHz / 8 = 6.048 MHz pixel clock, 384 x 264 total, 288 x 224 visible, 59.659 Hz
(identical to NB-1). MAME's `-listxml` confirms both sets: ROT0, 288 x 224.

## 2. Main CPU memory map [SRC 1072-1094] with observed use [OBS]

68EC020: 24 address lines. All windows are decoded by MAME exactly as below; everything else is unmapped
(reads return 0 in MAME, writes are dropped).

| Address | Device | Width / notes | Observed use |
| --- | --- | --- | --- |
| `$000000-$0FFFFF` | program ROM | 32-bit, `ROM_LOAD32_WORD` (mpru bytes 0-1, mprl bytes 2-3) | reset vectors: see section 3 |
| `$1C0000-$1CFFFF` | RAM | | **never accessed** by either game in 9,000 frames |
| `$1E4000-$1E4003` | random generator / seed | `machine().rand()` per read | not accessed |
| `$200000-$207FFF` | shared RAM (C75 `$4000-$BFFF`) | 32-bit handler over 16-bit RAM: high word = even word | `$200000-$200FFF` and `$206000-$206FFF` only |
| `$208000-$2FFFFF` | work RAM | | see section 2.1 |
| `$400000-$4FFFFF` | **data ROM** (`data` region, 1 MiB) | 32-bit BE, `ROM_LOAD16_WORD_SWAP` | read sparsely by both games |
| `$600000-$61FFFF` | C355 sprite RAM | as NB-1 | `$600000-$610FFF`, `$613000-$614FFF`, `$618000`, `$61A000`, `$61F000` |
| `$620000-$620007` | C355 position | as NB-1 | |
| `$640000-$64000F` | RAM, "unknown xy offset" | plain RAM in MAME | written by both games |
| `$680000-$68FFFF` | C123 tile VRAM | as NB-1 `$640000` | `$680000-$689FFF` |
| `$6C0000-$6C003F` | C123 control | as NB-1 `$660000` | |
| `$700000-$71FFFF` | **C169 ROZ VRAM** (64K words) | | whole window written by both games |
| `$740000-$74001F` | **C169 control** (16 words) | | |
| `$800000-$807FFF` | C116 palette + registers | as NB-1 `$700000` | |
| `$900008-$90000F` | sprite tile bank (8 bytes, RAM) | NB-1: `$680000-$68000F` | |
| `$940000-$94000F` | **tile bank** (16 bytes, RAM) | NB-2 only | |
| `$980000-$98000F` | **ROZ bank** (16 bytes, RAM; a change re-dirties the ROZ tilemaps in MAME) | NB-2 only | |
| `$A00000-$A007FF` | EEPROM 2816 | as NB-1 `$580000` | |
| `$C00000-$C0001F` | KEYCUS (read), writes ignored | as NB-1 `$6E0000` | |
| `$F00000-$F0001F` | cpureg (byte registers) | different order from NB-1 `$400000` | section 4 |

Unmapped accesses seen [OBS]: Mach Breakers reads `$56C000`, `$59B000`, `$5AB000` (16 bytes each, a few frames) and
both games write `$260000` / `$27C000` once at reset (work RAM in MAME, never read).

### 2.1 Work RAM use [OBS, 9,000 frames with autoplay]

Both games write the whole of `$208000-$240FFF` in their boot RAM clear (frames 25-28), then use:

* Outfoxies: `$208000-$21FFFF` heavily; `$220000-$234FFF` cleared repeatedly up to frame 1,225 and read a few hundred
  times per page; `$235000-$236FFF`, `$238000-$238FFF`, `$240000-$240FFF` heavily (the stack is in `$240xxx`).
* Mach Breakers: `$208000-$20BFFF`, `$218000-$219FFF`, `$21B000`, `$21E000`, `$23A000`, `$23E000-$240FFF`.

Nothing above `$241000` except the two reset writes. Reads of the remaining pages after boot are rare, but a later
stage may use them, so all of `$208000-$240FFF` must be backed.

## 3. Reset and boot [OBS]

| Step | Outfoxies | Mach Breakers |
| --- | --- | --- |
| Early cpureg set-up (`$F00008-$F00013`, `$F0001C-1D` = values below) | frame 0 | frame 0 |
| C75 held (`$F00016` = 0) | frame 0 | frame 0 |
| EEPROM read (`$A00000` = `$FF` on first boot -> defaults written) | F26 | F23 |
| KEYCUS: read word 1 = `$0186`, read word 2 three times (must change) | F26 | (no KEYCUS) |
| C75 released (`$F00016` = 1) | F26 | F23 L164 |
| C75 BIOS handshake: C75 writes `$A000 = $0080` (68020 `$206001` bit 7), 68020 answers, C75 clears it | F26 L224 | F24 L33 |
| VBL level written | `$F00000` = 1 (F26) | `$F00000` = 1 (F23) |
| POS level | `$F00002` = 5 | `$F00002` = 0 (raster IRQ unused) |
| Per frame | VBL ack `$F00004` and POS ack `$F00006` | VBL ack, POS level rewritten 0 |

cpureg values written at reset (identical in both games except where shown):
`$F00009=$62` (MAME's comment; Mach Breakers writes `$61` first), `$F0000A=$0F`, `$F0000B=$41`,
`$F0000C/D=$00` then `$70` (Outfoxies), `$F0000E=$23`, `$F0000F=$50`, `$F00010=$00`, `$F00011=$64`, `$F00012=$18`,
`$F00013=$E7`, `$F0001C=$44`/`$00`, `$F0001D=$11`/`$01`. Their function is unknown [SRC "???"]; MAME ignores them.

The 68020 kicks the watchdog (`$F00014`) and writes the KEYCUS (`$C00000`, Outfoxies) inside its delay loops; MAME
ignores both. Outfoxies reads `$F00000-$F0001F` about 9,000 times per frame (all return `$FF` [SRC 893-900]).

## 4. Interrupts

68EC020 [SRC 796-880]: autovectored levels from byte registers.

| Register | Function |
| --- | --- |
| `$F00000` | VBL IRQ level (bits 3-0); writing it also clears the pending VBL |
| `$F00001` | "???" IRQ level (never raised by MAME) |
| `$F00002` | POS (C116 raster) IRQ level |
| `$F00004` | VBL ack |
| `$F00005` | "???" ack |
| `$F00006` | POS ack |
| `$F00014` | watchdog (not modelled by MAME) |
| `$F00016` | C75 control: bit 0 = 1 clears HALT and pulses RESET; 0 = HALT |
| reads | every byte reads `$FF` |

Raise points [SRC 666-686] (same scantimer as NB-1): VBL at scanline 224; POS when scanline == C116 reg 5 - 32. The
MAME driver keeps `m_update_to_line_before_posirq = false` for NB-2 (only Nebulas Ray sets it).

C75 [SRC 1346-1347]: IRQ0 and IRQ2 from two free-running 60 Hz timers (MAME TODO "Real sources"), as on NB-1. The
NB-1 core's C75 timing (`nb1_c75_irq`, the timer A0 count-stop fix) applies unchanged [NB1].

## 5. C75 and shared RAM [SRC 1096-1166] [NB1] [OBS]

C75 map: `$2000-$2FFF` C352, `$4000-$BFFF` shared RAM (68020 `$200000-$207FFF`), `$C000-$FFFF` internal BIOS,
`$200000-$27FFFF` data ROM (`c75data` region, 512 KiB, 16-bit little-endian; both games use only
`$200000-$226FFF` in 9,000 frames). Unmapped write `$150000` once per frame (BIOS, as NB-1). Ports P6/P7/AN0-7 for
the inputs exactly as NB-1 (section 8). The BIOS handshake at `$A000` and the per-frame mailbox are the same as on
NB-1; both games release the C75 once and keep it running.

## 6. Video

### 6.1 Composition [SRC 600-630]

MAME draws, for `pri` = 0..15: both ROZ layers whose priority field equals `pri` (layer 1 first, then layer 0), then
(even `pri` only) the C123 layers whose priority equals `pri/2`, in layer order 0..5. Later opaque pixels overwrite
earlier ones. The screen priority map receives `pri` from ROZ pixels and `pri` (= 2 x the C123 priority) from tile
pixels. Sprites then mix per pixel against that map exactly as NB-1 (`sprite_mix_callback` [SRC 530-549]): a sprite
pixel with priority >= the map wins; pen `$FFE` is a shadow (`dest |= $800`).

Equivalent single-pass rule used by the core: give each opaque background pixel the key
`(pri, stage, order)` with ROZ = `(P, 0, 1 - layer)` and C123 = `(2p, 1, layer)`; the visible background pixel is
the opaque one with the largest key; its priority-map value is P (ROZ) or 2p (C123), 0 where nothing is opaque.

Pens: sprites `$000-$FFF`; C123 `$1000 + bank*256 + pixel`; ROZ `$1800 + color*256 + pixel` (color 4 bits). The C116
palette has `$2000` pens; ROZ colours 8-15 would address MAME's shadow pens (`$2000+`). Observed ROZ colours: 0-2.

Clip window [SRC 603-608]: X = reg0 - `$4B` .. reg1 - `$4B` - 1 (NB-1: `$4A`), Y = reg2 - `$21` .. reg3 - `$21` - 1.

### 6.2 C123 tilemaps (as NB-1, except the tile callback)

Callbacks give the CHR (pixel) tile number and the SHAPE (mask) tile number from the 16-bit VRAM code:

* Outfoxies [SRC 495-500]: `tile = bitswap16(code, 15..9, 6, 7, 8, 5..0)` (bits 6 and 8 exchanged), `mask = code`.
* Mach Breakers [SRC 485-493]: `bank = tilebank byte[(code >> 13) + 8]` (bytes `$940008-$94000F`),
  `tile = mask = (code & $1FFF) | (bank << 13)`. Observed bank bytes: identity `00..07` throughout.

MAME takes the CHR tile modulo the number of 8x8 tiles in the region (Outfoxies 32,768; Mach Breakers 98,304); the
SHAPE pointer is `mask * 8` into a 2 MiB region (Mach Breakers' 1 MiB ROM leaves the upper half 0 = transparent).

### 6.3 C355 sprites (as NB-1, except the bank callback)

The bank array `$900008-$90000F` is copied at the start of vblank (`screen_vblank`) and used for the next sprite
build (NB-1 behaviour). Code = `(code & $7FF) | (B << 11)` with `b = bank byte[(code >> 11) & 15]` and
[SRC 632-646]:

* Mach Breakers: `B = bitswap6(b & $5F, 6, 4, 3, 2, 1, 0)`.
* Outfoxies: `B = bitswap6(b & $5F, 6, 4, 3, 1, 2, 0)`.

Bank indices 8-15 read past the 8-byte array in MAME (undefined); observed codes stay below `$4000`. Observed bank
bytes include `$40`-`$46` (bit 6 in use) [OBS].

### 6.4 C169 ROZ [SRC namco_c169roz.cpp]

* VRAM: 64K words (`set_ram_words(0x20000 / 2)`). Two 256 x 256-tile maps of 16 x 16 tiles share it; tile index
  `((col & $80) << 8) | (row << 7) | (col & $7F)`, code = word & `$3FFF`.
* Control: 16 words, 8 per layer (layer 0 words 0-7, layer 1 words 8-15). Word 1: bit 15 disable, bit 11 wrap
  (MAME ignores wrap disable), bits 9-8 size (512 << n), bits 7-4 priority, bits 3-0 colour. Words 2-5: `incxx`,
  `incxy`, `incyx`, `incyy` (12-bit signed with sign in bit 15, bits 14-12 of words 2/3 = left/top in 512-pixel
  units). Words 6-7: start X/Y (16-bit signed, << 4). MAME adds 36 x inc / 3 x inc offsets.
* Per-line mode: when control word 0 == `$8000`, layer 1 takes its 8 parameter words per scanline from VRAM at
  byte `$E080 + (line >> 3) * $100 + (line & 7) * $10`. Mach Breakers uses this mode for its floor [OBS
  `$740000 = $8000`]; Outfoxies keeps word 0 = `$1000` [OBS].
* Pixel: `xpos = ((cx >> 16) & (size - 1)) + left) & $FFF` (same for y); a pixel is drawn if its mask bit is set;
  pen `$1800 + colour*256 + pixel`; priority value = the layer's priority field.
* Tile callbacks [SRC 502-518]: `bank = rozbank byte[layer*8 + ((code >> 11) & 7)]`,
  `mangle = (code & $7FF) | (bank << 11)`; Mach Breakers `tile = mask = mangle`; Outfoxies `mask = mangle`,
  `tile = bitswap19(mangle, 18..7, 4, 5, 6, 3..0)` (bits 4 and 6 exchanged).
* Graphics: 16 x 16 x 8 bpp, 256 bytes per tile (Outfoxies 6 MiB = 24,576 tiles, Mach Breakers 4 MiB = 16,384);
  mask 32 bytes per tile (2 MiB / 512 KiB ROMs in 2 MiB regions).
* Observed use [OBS]: both layers enabled in gameplay. Outfoxies scales without rotation (`incxx` = `incyy` from
  `$0101` to `$03FF`, i.e. up to 4 source pixels per screen pixel, both layers equal, layer 1 64 tiles lower).
  Mach Breakers uses rotation/scale on layer 0 and the per-line mode on layer 1.

Mask analysis [OBS, offline over the ROM data]: every pixel value 0-255 occurs as an opaque pixel in both games' ROZ
and C123 graphics, so the mask ROMs carry information the pixel data does not; they must be read.

## 7. Sound [SRC] [OBS]

C352 as NB-1 [NB1]. Voice region (16 MiB address space): Outfoxies `ou1voi0` at 0 (2 MiB), rest 0. Mach Breakers
`mb1_voi0` at 0 and `$400000` (reload), `mb1_voi1` at `$800000` and `$C00000` (reload). Voice banks written in the
first 1,500 frames: Outfoxies `$0A-$0E`; Mach Breakers `$01`, `$0A`, `$90-$9E` (address bit 22 never set).

## 8. Inputs, switches, EEPROM

Identical to NB-1 [SRC 1202-1256]: four players, 8-way stick, 3 buttons and start each (active low), four coins,
service coin, test switch, service-mode DIP (SW1-1) and freeze DIP (SW1-2), through the C75 ports. The EEPROM is a
2816 at `$A00000`; both games find it erased on first boot and write their defaults (Outfoxies F26, Mach Breakers F23)
[OBS]. No default EEPROM image exists in MAME.

## 9. ROM regions [SRC 2175-2216, 2343-2386]

| Region | Outfoxies | Mach Breakers |
| --- | --- | --- |
| `maincpu` 1 MiB | `ou2_mpru.11d` (bytes 0-1), `ou2_mprl.11c` (2-3) | `mb2_mpru.11d`, `mb2_mprl.11c` |
| `c75data` 512 KiB LE | `ou1spr0.5b` | `mb1_spr0.5b` |
| `c352` 16 MiB | `ou1voi0.6n` @0 (2 MiB) | `mb1_voi0.6n` @0 + @`$400000`, `mb1_voi1.6p` @`$800000` + @`$C00000` |
| `c123tmap:mask` 2 MiB | `ou1shas.12s` (2 MiB) | `mb1_shas.12s` (1 MiB) |
| `c169roz:mask` 2 MiB | `ou1shar.18s` (2 MiB) | `mb1_shar.18s` (512 KiB) |
| `c355spr` 32 MiB | `ou1obj0l..4u`: 5 x (L bytes 0-1, U bytes 2-3), 20 MiB | `mb1obj0l..4u`, 20 MiB |
| `c169roz` | 6 MiB: `ou1-rot0/1/2` | 4 MiB: `mb1_rot0/1` |
| `c123tmap` | 2 MiB: `ou1-scr0` | 6 MiB: `mb1_scr0/1/2` |
| `data` 1 MiB BE | `ou1dat0.20a`, `ou1dat1.20b` (16-bit word-swapped) | `mb1_dat0.20a` (512 KiB) |
| C75 BIOS | `c75.bin` (16 KiB, `namcoc75.zip`) | same |

## 10. Open questions

| # | Question | Plan |
| --- | --- | --- |
| Q1 | `$640000-$64000F` "unknown xy offset" | plain RAM as MAME; look for a visible effect in captures |
| Q2 | KEYCUS C390 behaviour beyond MAME's ID + changing word | MAME model (NB-1 generic KEYCUS) |
| Q3 | C169 wrap disable (bit 11), not modelled by MAME | follow MAME; check captures for layers with wrap off |
| Q4 | Sprite bank index 8-15 | check a long capture for codes >= `$4000` |
| Q5 | cpureg `$F00008-$F00013`, `$F0001C-1D` | ignore as MAME |
