#!/usr/bin/env python3
# Namco NB-2 MiSTer core -- MRA generator and validator (M2).
# Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
#
# Derived from the NB-1 core's scripts/mra/nb1_mra.py (provenance: docs/PROVENANCE.md). NB-2 differences:
# the platform map (docs/ARCHITECTURE.md section 2.3: SDRAM + DDR3), ROM_LOAD16_WORD_SWAP and ROM_RELOAD,
# window transforms (VOICE drops C352 address bit 22), the "swap16" check lane mode, and the NB2B board
# record (KEYCUS + bank-transform flags + hot work-RAM pages).
#
# Subcommands
#   extract   MAME source (ROM_START) + `mame -listxml` -> games/<set>.json (ROM metadata only) plus the board
#             description derived from MAME (KEYCUS from custom_key_r, bank transforms from the machine config).
#             The hot work-RAM pages are not in MAME's source: they come from --hot-pages (a MAME profile
#             summary, docs/NB2_HARDWARE_REFERENCE.md section 2.1).
#   generate  games/<set>.json -> MRA (index 3 check record, index 2 board record, index 1 EEPROM, index 0 ROMs)
#   validate  MRA + games/<set>.json [+ --listxml] [+ --zip] [+ --mame-regions DIR]:
#             1. rebuild the index-0 stream as MiSTer's mra_loader.cpp does from SYNTHETIC file contents and
#                compare it byte for byte with MAME's region assembly placed into the NB-2 map;
#             2. names/sizes/CRCs against MAME metadata, the map against rtl/nb2/nb2_mem_pkg.sv, the check
#                record's coverage and CRCs, the board record against the JSON (and MAME with --mame-src);
#             3. --zip: the real stream assembled in memory from the owner's ROM set, every check CRC on it;
#             4. --mame-regions: region dumps from MAME itself (scripts/mame/nb2_regions.lua) -- every window
#                of the real stream must equal MAME's region memory through the window transform.
#             Nothing derived from ROM contents is written anywhere.
import argparse, binascii, hashlib, json, os, re, struct, sys, zipfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))

MiB = 0x100000
# (name, region id, stream/physical base, window, MAME region tag, transform)
# transform: list of (window offset, MAME region offset, length) slices; None = identity [0, window)
PLATFORM_MAP = [
    ('ROZ',      8, 0x0000000, 8 * MiB,   'c169roz',       None),
    ('CHR',      4, 0x0800000, 8 * MiB,   'c123tmap',      None),
    ('VOICE',    2, 0x1000000, 8 * MiB,   'c352',          [(0, 0, 4 * MiB), (4 * MiB, 8 * MiB, 4 * MiB)]),
    ('SHAPE',    5, 0x1800000, 2 * MiB,   'c123tmap:mask', None),
    ('ROZMASK',  9, 0x1A00000, 2 * MiB,   'c169roz:mask',  None),
    ('PROG',     0, 0x1C00000, 1 * MiB,   'maincpu',       None),
    ('DATA',     7, 0x1D00000, 1 * MiB,   'data',          None),
    ('C75DATA',  1, 0x1E00000, 0x80000,   'c75data',       None),
    ('C75BIOS',  6, 0x1E80000, 0x4000,    'mcu:internal',  None),
    ('GAP',     15, 0x1E84000, 0x017C000, None,            None),   # zero fill up to the DDR3 part
    ('OBJ',      3, 0x2000000, 20 * MiB,  'c355spr',       None),   # DDR3
]
STREAM_END = 0x3400000
DDR3_STREAM_BASE = 0x2000000
IOCTL_ROM, IOCTL_NVRAM, IOCTL_BOARD, IOCTL_CHECK = 0, 1, 2, 3
NVRAM_BYTES = 2048
RBF = 'Namco_NB2'
MAX_ENTRIES = 32
LANES_SWAP16 = 0x12      # check entry lane code: one CRC over bytes 1,0,3,2 of every longword

WRAM_PAGE0, WRAM_PAGES = 0x208, 57    # 4 KiB pages $208000-$240FFF

def fail(msg):
    print('FAIL:', msg)
    sys.exit(1)

# ---------------------------------------------------------------------------
# board description from MAME
KC_CASE_RE = re.compile(r'\bcase\s+(NAMCONB[12]_\w+)\s*:(.*?)(?=\bcase\s+NAMCONB[12]_|\n\t\}\n)', re.S)
KC_OFF_RE = re.compile(r'\bcase\s+(\d+)\s*:\s*return\s+([^;]+);')
KC_NONE = {'mode': 0, 'id': 0, 'id_word': 0, 'rnd_word': 0}

def keycus_types(src):
    body = src[src.index('::custom_key_r('):]
    body = body[:body.index('\n}\n')]
    out = {}
    for gt, blk in KC_CASE_RE.findall(body):
        words = {}
        for off, expr in KC_OFF_RE.findall(blk):
            n = int(off)
            for term in expr.replace('(', ' ').replace(')', ' ').split('|'):
                t = ' '.join(term.split())
                hi = t.endswith('<< 16')
                v = t[:-5].strip() if hi else t
                w = 2 * n + (0 if hi else 1)
                if v == 'm_count':
                    words[w] = 'rnd'
                elif re.fullmatch(r'0x[0-9a-fA-F]+|\d+', v):
                    if int(v, 0) != 0:
                        words[w] = int(v, 0)
                else:
                    fail('custom_key_r %s: cannot parse %r' % (gt, t))
        ids = [(w, v) for w, v in words.items() if v != 'rnd']
        rnd = [w for w, v in words.items() if v == 'rnd']
        if len(ids) > 1 or len(rnd) > 1 or (bool(ids) != bool(rnd)):
            fail('custom_key_r %s does not fit the ID + changing-word model: %r' % (gt, words))
        out[gt] = {'mode': 1, 'id': ids[0][1], 'id_word': ids[0][0], 'rnd_word': rnd[0]} if ids else dict(KC_NONE)
    return out

def game_line(src, setname):
    m = re.search(r'\bGAME\(\s*\d+\s*,\s*%s\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*\w+\s*,\s*(\w+)\s*,\s*init_(\w+)\s*,' % re.escape(setname), src)
    if not m:
        fail('no GAME() line for ' + setname)
    return {'parent': m.group(1), 'machine': m.group(2), 'state': m.group(3), 'init': m.group(4)}

def keycus_from_source(src, setname):
    init = game_line(src, setname)['init']
    g = re.search(r'::init_%s\(\)\s*\{[^}]*?m_gametype\s*=\s*(\w+)\s*;' % init, src)
    if not g:
        fail('init_%s sets no m_gametype' % init)
    kc = dict(keycus_types(src).get(g.group(1)) or KC_NONE)
    kc['part'] = ('C%d' % kc['id']) if kc['mode'] else ''
    kc['source'] = 'custom_key_r %s (init_%s)' % (g.group(1), init)
    return kc

# The bank transforms are chosen by the machine config (outfxies / machbrkr callbacks), not the game init:
# outfxiesja runs the machbrkr config. Flags (board record byte 13) [docs/NB2_HARDWARE_REFERENCE.md 6.2-6.4]:
#   bit 0 TILE_SWAP68  C123 pixel tile = code with bits 6 and 8 exchanged (mask tile = code)
#   bit 1 TILE_BANK    C123 tile = mask = (code & $1FFF) | tilebank[(code >> 13) + 8] << 13
#   bit 2 ROZ_SWAP46   ROZ pixel tile = mangle with bits 4 and 6 exchanged (mask tile = mangle)
#   bit 3 SPR_SWAP12   sprite bank bitswap (6,4,3,1,2,0) instead of (6,4,3,2,1,0)
CFG_FLAGS = {'outfxies': 0x1 | 0x4 | 0x8, 'machbrkr': 0x2}

def board_from_source(src, setname):
    gl = game_line(src, setname)
    cfg = gl['machine']
    if cfg not in CFG_FLAGS:
        fail('machine config %s has no NB-2 transform description' % cfg)
    # cross-check the callbacks the config installs against what the flags stand for
    body = re.search(r'void namconb2_state::%s\(machine_config &config\)\s*\{(.*?)\n\}' % cfg, src, re.S).group(1)
    for dev, cb in (('c123tmap', '%s_tilemap_cb' % cfg), ('c169roz', '%s_roz_cb' % cfg), ('c355spr', '%s_objcode2tile_cb' % cfg)):
        if cb not in body:
            fail('%s config does not install %s' % (cfg, cb))
    return {'keycus': keycus_from_source(src, setname), 'flags': CFG_FLAGS[cfg], 'machine': cfg}

# ---------------------------------------------------------------------------
# extract
LOAD_RE = re.compile(r'\b(ROM_LOAD(?:32_WORD|32_BYTE|16_BYTE|16_WORD_SWAP)?)\s*\(\s*"([^"]+)"\s*,\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*(0x[0-9a-fA-F]+)\s*,\s*CRC\(([0-9a-fA-F]+)\)\s*SHA1\(([0-9a-fA-F]+)\)')
RELOAD_RE = re.compile(r'\bROM_RELOAD\s*\(\s*(0x[0-9a-fA-F]+)\s*,\s*(0x[0-9a-fA-F]+)\s*\)')
REGION_RE = re.compile(r'\bROM_REGION(\w*)\s*\(\s*(0x[0-9a-fA-F]+)\s*,\s*"([^"]+)"\s*,\s*([^)]*)\)')

def parse_rom_start(src, setname):
    m = re.search(r'ROM_START\(\s*%s\s*\)(.*?)ROM_END' % re.escape(setname), src, re.S)
    if not m:
        fail('ROM_START(%s) not found' % setname)
    body = m.group(1)
    if re.search(r'ROM_(CONTINUE|FILL|COPY|IGNORE)\b', body):
        fail('unsupported ROM macro in ROM_START(%s)' % setname)
    regions, cur = [], None
    for line in body.splitlines():
        line = line.split('//')[0]
        r = REGION_RE.search(line)
        if r:
            flags = r.group(4)
            cur = {'tag': r.group(3), 'size': int(r.group(2), 16), 'width': r.group(1) or '',
                   'fill': 0xFF if 'ERASEFF' in flags else 0x00, 'loads': []}
            regions.append(cur)
            continue
        l = LOAD_RE.search(line)
        if l:
            cur['loads'].append({'kind': l.group(1), 'name': l.group(2), 'offset': int(l.group(3), 0),
                                 'length': int(l.group(4), 16), 'crc': l.group(5).lower(),
                                 'sha1': l.group(6).lower()})
            continue
        rl = RELOAD_RE.search(line)
        if rl:
            prev = cur['loads'][-1]
            if prev['kind'] != 'ROM_LOAD' or int(rl.group(2), 16) != prev['length']:
                fail('unsupported ROM_RELOAD after %s' % prev['name'])
            d = dict(prev)
            d['offset'] = int(rl.group(1), 16)
            d['reload'] = True
            cur['loads'].append(d)
        elif 'ROM_LOAD' in line or 'ROM_RELOAD' in line:
            fail('unparsed ROM line: ' + line.strip())
    return regions

def parse_hot_pages(s):
    pages = sorted({int(p, 16) for p in s.replace(',', ' ').split()})
    for p in pages:
        if not (WRAM_PAGE0 <= p < WRAM_PAGE0 + WRAM_PAGES):
            fail('hot page %03X outside $208000-$240FFF' % p)
    return ['%03X' % p for p in pages]

def cmd_extract(a):
    src = open(a.mame_src, encoding='utf-8').read()
    regions = parse_rom_start(src, a.set)
    root = ET.parse(a.listxml).getroot()
    mach = {m.get('name'): m for m in root.iter('machine')}
    if a.set not in mach:
        fail('%s not in listxml' % a.set)
    g = mach[a.set]
    lx = {}
    for r in g.findall('rom'):
        lx.setdefault(r.get('name'), []).append(r)
    nloads = 0
    for reg in regions:
        for ld in reg['loads']:
            if ld.get('reload'):
                continue    # -listxml lists a reloaded file once, at its first offset
            nloads += 1
            cands = lx.get(ld['name'], [])
            if not any(r.get('region') == reg['tag'] and int(r.get('size')) == ld['length'] and r.get('crc') == ld['crc']
                       and r.get('sha1') == ld['sha1'] and int(r.get('offset'), 16) == ld['offset'] for r in cands):
                fail('listxml disagrees with source for %s at 0x%X' % (ld['name'], ld['offset']))
    if sum(len(v) for v in lx.values()) != nloads:
        fail('listxml lists ROMs the source parse missed')
    dev = {d.get('tag'): d.get('name') for d in g.findall('device_ref')}
    c75 = mach.get(dev.get(':mcu', ''))
    if c75 is None:
        fail('C75 device entry missing from listxml (add namcoc75 to the -listxml call)')
    b = c75.find('rom')
    regions.append({'tag': 'mcu:internal', 'size': int(b.get('size')), 'width': '', 'fill': 0,
                    'device': c75.get('name'),
                    'loads': [{'kind': 'ROM_LOAD', 'name': b.get('name'), 'offset': 0,
                               'length': int(b.get('size')), 'crc': b.get('crc'), 'sha1': b.get('sha1')}]})
    disp = g.find('display')
    board = board_from_source(src, a.set)
    board['hot_pages'] = parse_hot_pages(a.hot_pages)
    board['hot_pages_source'] = a.hot_pages_note or ''
    out = {
        'set': a.set, 'parent': g.get('cloneof') or '', 'description': g.findtext('description'),
        'year': g.findtext('year'), 'manufacturer': g.findtext('manufacturer'),
        'rotate': int(disp.get('rotate', '0')),
        'mame': {'version': root.get('build'), 'source': os.path.basename(a.mame_src), 'note': a.note or ''},
        'regions': regions,
        'board': board,
    }
    with open(a.out, 'w', newline='\n') as f:
        json.dump(out, f, indent=1)
        f.write('\n')
    print('wrote', a.out)

# ---------------------------------------------------------------------------
# MAME region assembly and platform placement
def mame_region(reg, files):
    buf = bytearray([reg['fill']]) * reg['size']
    for ld in reg['loads']:
        d = files[ld['name']]
        o, n, k = ld['offset'], ld['length'], ld['kind']
        if k == 'ROM_LOAD':
            buf[o:o+n] = d
        elif k == 'ROM_LOAD32_WORD':
            for i in range(0, n, 2):
                buf[o + 2*i: o + 2*i + 2] = d[i:i+2]
        elif k == 'ROM_LOAD16_WORD_SWAP':
            sw = bytearray(n)
            sw[0::2] = d[1::2]
            sw[1::2] = d[0::2]
            buf[o:o+n] = sw
        else:
            fail('unsupported load kind ' + k)
    return buf

def load_span(ld):
    """(region start, region length) a load occupies."""
    k, o, n = ld['kind'], ld['offset'], ld['length']
    if k == 'ROM_LOAD32_WORD':
        return o & ~3, 2 * n
    return o, n

def slices(entry, reg_size):
    name, rid, base, size, tag, xf = entry
    return xf if xf else [(0, 0, size)]

def window_bytes(entry, img):
    """The window contents from a MAME region image (bytes beyond the region read as 0)."""
    name, rid, base, size, tag, xf = entry
    out = bytearray(size)
    for wo, ro, n in slices(entry, len(img)):
        seg = img[ro:ro+n]
        out[wo:wo+len(seg)] = seg
    return out

def platform_stream(game, files):
    regs = {r['tag']: r for r in game['regions']}
    out = bytearray()
    for e in PLATFORM_MAP:
        name, rid, base, size, tag, xf = e
        if len(out) != base:
            fail('map not contiguous at ' + name)
        reg = regs.get(tag) if tag else None
        if reg is None:
            out += bytes(size)
            continue
        img = mame_region(reg, files)
        # data outside the window must be MAME fill (otherwise the window loses information)
        covered = bytearray(len(img))
        for wo, ro, n in slices(e, len(img)):
            covered[ro:ro+n] = b'\x01' * min(n, max(0, len(img) - ro))
        for ld in reg['loads']:
            s, n = load_span(ld)
            if ld.get('reload') and xf:
                continue   # a reload is a mirror the window transform folds away
            if not all(covered[s:s+n]):
                fail('%s: %s at 0x%X lies outside the %s window' % (tag, ld['name'], ld['offset'], name))
        out += window_bytes(e, img)
    if len(out) != STREAM_END:
        fail('stream length 0x%X' % len(out))
    return out

# ---------------------------------------------------------------------------
# window plan: which file (or fill) supplies every window byte, in window order
def window_plan(entry, reg):
    """List of ('fill', length) / ('load', load) / ('group', [loads]) covering the window in order."""
    name, rid, base, size, tag, xf = entry
    plan = []
    pos = 0
    fillb = reg['fill'] if reg else 0
    if reg:
        for wo, ro, n in slices(entry, reg['size']):
            # loads that lie inside [ro, ro+n), skipping mirrors folded by the transform
            inside = []
            for ld in reg['loads']:
                s, ln = load_span(ld)
                if s >= ro and s + ln <= ro + n:
                    inside.append(ld)
            inside.sort(key=lambda l: l['offset'])
            i = 0
            while i < len(inside):
                ld = inside[i]
                s, ln = load_span(ld)
                w = wo + (s - ro)
                if w < pos:
                    fail('%s: overlapping loads' % name)
                if w > pos:
                    plan.append(('fill', w - pos, fillb))
                    pos = w
                if ld['kind'] == 'ROM_LOAD32_WORD':
                    grp = inside[i:i+2]
                    if len(grp) != 2 or grp[1]['offset'] != ld['offset'] + 2 or grp[1]['kind'] != ld['kind'] \
                       or grp[1]['length'] != ld['length']:
                        fail('%s: incomplete ROM_LOAD32_WORD pair' % ld['name'])
                    plan.append(('group', grp))
                    i += 2
                else:
                    plan.append(('load', ld))
                    i += 1
                pos += ln
            end = wo + min(n, max(0, reg['size'] - ro))
            if end < wo + n:   # window part beyond the MAME region: 0 (reads past a region)
                pass
    if pos < size:
        plan.append(('fill', size - pos, fillb if reg else 0))
    return plan

# ---------------------------------------------------------------------------
# Check record
def crc32(b):
    return binascii.crc32(b) & 0xFFFFFFFF

def check_entries(game):
    regs = {r['tag']: r for r in game['regions']}
    ents = []
    for e in PLATFORM_MAP:
        name, rid, base, size, tag, xf = e
        if tag is None:
            continue    # the gap is not checked
        reg = regs.get(tag)
        pos = 0
        for item in window_plan(e, reg):
            if item[0] == 'fill':
                n, fb = item[1], item[2]
                ents.append((rid, 1, pos, n, [crc32(bytes([fb]) * n)]))
                pos += n
            elif item[0] == 'load':
                ld = item[1]
                if pos % 4 or ld['length'] % 4:
                    fail('%s: unaligned load' % ld['name'])
                lanes = LANES_SWAP16 if ld['kind'] == 'ROM_LOAD16_WORD_SWAP' else 1
                ents.append((rid, lanes, pos, ld['length'], [int(ld['crc'], 16)]))
                pos += ld['length']
            else:
                grp = item[1]
                n = 2 * grp[0]['length']
                ents.append((rid, 2, pos, n, [int(g['crc'], 16) for g in grp]))
                pos += n
        if pos != size:
            fail('%s: plan covers 0x%X of 0x%X' % (name, pos, size))
    if len(ents) > MAX_ENTRIES:
        fail('too many check entries (%d)' % len(ents))
    return ents

def record_bytes(ents, stream_len):
    b = bytearray(b'NB2C' + bytes([1, len(ents), 0, 0]) + struct.pack('<I', stream_len))
    b += bytes(32 - len(b))
    for rid, lanes, o, n, crcs in ents:
        c = (crcs + [0, 0, 0, 0])[:4]
        b += bytes([rid, lanes, 0, 0]) + struct.pack('<IIIIII', o, n, *c) + bytes(4)
    return bytes(b)

def parse_record(b):
    if b[:4] != b'NB2C' or b[4] != 1:
        fail('check record header')
    n = b[5]
    (slen,) = struct.unpack_from('<I', b, 8)
    ents = []
    for i in range(n):
        e = b[32 + 32*i: 64 + 32*i]
        rid, lanes = e[0], e[1]
        o, ln, c0, c1, c2, c3 = struct.unpack_from('<IIIIII', e, 4)
        ents.append((rid, lanes, o, ln, [c0, c1, c2, c3][:(1 if lanes == LANES_SWAP16 else lanes)]))
    return slen, ents

def entry_crcs(stream, base, o, n, lanes):
    """What the core's checker computes."""
    seg = stream[base + o: base + o + n]
    if lanes == LANES_SWAP16:
        sw = bytearray(n)
        sw[0::2] = seg[1::2]
        sw[1::2] = seg[0::2]
        return [crc32(sw)]
    if lanes == 1:
        return [crc32(seg)]
    per = 4 // lanes
    return [crc32(b''.join(seg[i + j*per: i + j*per + per] for i in range(0, n, 4))) for j in range(lanes)]

# ---------------------------------------------------------------------------
# board record (NB2B v1, 32 bytes, little-endian)
HOT_SLOTS = 28                                  # rtl/nb2/nb2_main_bus.sv SLOTS

def board_bytes(game):
    bd = game['board']
    kc = bd['keycus']
    if kc['mode'] not in (0, 1) or not (0 <= kc['id_word'] < 16) or not (0 <= kc['rnd_word'] < 16) \
       or not (0 <= kc['id'] < 0x10000):
        fail('board.keycus out of range')
    if kc['mode'] and kc.get('part') != 'C%d' % kc['id']:
        fail('board.keycus part does not match the ID')
    if game.get('rotate', 0) not in (0, 90, 180, 270):
        fail('rotate')
    if len(bd['hot_pages']) > HOT_SLOTS:
        fail('%d hot pages; the core has %d block-RAM slots (nb2_main_bus SLOTS)' % (len(bd['hot_pages']), HOT_SLOTS))
    mask = 0
    for p in bd['hot_pages']:
        mask |= 1 << (int(p, 16) - WRAM_PAGE0)
    b = b'NB2B' + bytes([1, 32, 0, 0, kc['mode'], kc['id_word']]) + struct.pack('<H', kc['id']) \
        + bytes([kc['rnd_word'], bd['flags'], game.get('rotate', 0) // 90, 0]) + struct.pack('<Q', mask) + bytes(8)
    assert len(b) == 32
    return b

# ---------------------------------------------------------------------------
# generate
def hexlines(b, indent):
    return '\n'.join(indent + ' '.join('%02X' % x for x in b[i:i+32]) for i in range(0, len(b), 32))

FAST_LOAD_ADDR = 0x30000000                     # rtl/nb2/nb2_fastload.sv BASE_W (stream offset 0 in DDR3)

def mra_zips(game):
    names = [game['set']] + ([game['parent']] if game.get('parent') else []) + ['namcoc75']
    return '|'.join(n + '.zip' for n in names)

def cmd_generate(a):
    game = json.load(open(a.game))
    regs = {r['tag']: r for r in game['regions']}
    ents = check_entries(game)
    rec = record_bytes(ents, STREAM_END)
    zips = mra_zips(game)
    bd = game['board']
    L = []
    L.append('<!--')
    L.append('  %s - Namco NB-2 - MRA for the %s MiSTer core.' % (game['description'], RBF))
    L.append('')
    L.append('  GENERATED by scripts/mra/nb2_mra.py from scripts/mra/games/%s.json' % game['set'])
    L.append('  (MAME %s metadata). Do not edit by hand; regenerate and run' % game['mame']['version'])
    L.append('  `nb2_mra.py validate`. No ROM data is embedded in this file.')
    L.append('')
    L.append('  Index 3: check record (the core verifies every region after loading).')
    L.append('  Index 2: board record (KEYCUS, graphics bank transforms, hot work-RAM pages).')
    L.append('  Index 1: EEPROM power-on image + MiSTer NVRAM save/restore.')
    L.append('  Index 0: the NB-2 platform ROM stream (docs/ARCHITECTURE.md section 2.3): every MAME region in')
    L.append('  MAME byte order, placed in its fixed window; stream offset == physical address (below')
    L.append('  0x2000000 SDRAM, above it the DDR3 sprite store). The C75 BIOS (c75.bin) comes from the game')
    L.append('  zip (non-merged sets) or from namcoc75.zip (split/merged sets).')
    L.append('  address="0x%08X": DDR3 fast loading (MiSTer copies the stream into DDR3 there and the core' % FAST_LOAD_ADDR)
    L.append('  replays it, rtl/nb2/nb2_fastload.sv); firmware without it streams the ROM as before.')
    L.append('-->')
    L.append('<misterromdescription>')
    L.append('    <name>%s</name>' % game['description'])
    L.append('    <setname>%s</setname>' % game['set'])
    L.append('    <rbf>%s</rbf>' % RBF)
    L.append('    <mameversion>%s</mameversion>' % re.sub(r'\D', '', game['mame']['version'])[:4].rjust(4, '0'))
    L.append('    <year>%s</year>' % game['year'])
    L.append('    <manufacturer>%s</manufacturer>' % game['manufacturer'])
    L.append('    <players>4</players>')
    L.append('    <joystick>8-way</joystick>')
    L.append('    <platform>Namco NB-2</platform>')
    L.append('    <buttons names="%s" default="A,B,X,Start,Select,R"/>'
             % game.get('buttons', 'Button 1,Button 2,Button 3,Start,Coin,Service'))
    L.append('')
    L.append('    <!-- check record: header + %d entries (region, lanes, offset, length, CRC32 per lane).' % len(ents))
    L.append('         Lane CRCs equal MAME\'s per-file CRC32s (lanes 0x12: one CRC over the 16-bit')
    L.append('         byte-swapped window = the file CRC of a ROM_LOAD16_WORD_SWAP file); gap entries carry')
    L.append('         the CRC of the fill. -->')
    L.append('    <rom index="%d">' % IOCTL_CHECK)
    L.append('        <part>')
    L.append(hexlines(rec, '            '))
    L.append('        </part>')
    L.append('    </rom>')
    L.append('')
    kc = bd['keycus']
    L.append('    <!-- board record (NB2B v1, 32 bytes): KEYCUS mode %d, ID $%04X (%s) on word %d, changing' % (
        kc['mode'], kc['id'], kc['part'] or 'none', kc['id_word']))
    L.append('         value on word %d (MAME %s); bank transforms $%02X (MAME %s config);' % (
        kc['rnd_word'], kc['source'], bd['flags'], bd['machine']))
    L.append('         hot work-RAM pages %s.' % ' '.join(bd['hot_pages']))
    L.append('         Format: rtl/nb2/nb2_board_config.sv. -->')
    L.append('    <rom index="%d">' % IOCTL_BOARD)
    L.append('        <part>')
    L.append(hexlines(board_bytes(game), '            '))
    L.append('        </part>')
    L.append('    </rom>')
    L.append('')
    L.append('    <!-- EEPROM (28C16, 2 KiB, MAME EEPROM_2816): erased power-on image (MAME has no default),')
    L.append('         then MiSTer replaces it with the saved .nvm and saves it after the game writes it. -->')
    if regs.get('eeprom'):
        fail('NB-2 sets have no default EEPROM region in MAME 0.289')
    L.append('    <rom index="%d">' % IOCTL_NVRAM)
    L.append('        <part repeat="0x%X">FF</part>' % NVRAM_BYTES)
    L.append('    </rom>')
    L.append('    <nvram index="%d" size="%d"/>' % (IOCTL_NVRAM, NVRAM_BYTES))
    L.append('')
    L.append('    <rom index="%d" zip="%s" md5="none" address="0x%08X">' % (IOCTL_ROM, zips, FAST_LOAD_ADDR))
    for e in PLATFORM_MAP:
        name, rid, base, size, tag, xf = e
        reg = regs.get(tag) if tag else None
        L.append('        <!-- %s: stream 0x%07X-0x%07X%s%s -->' % (name, base, base + size - 1,
                 (', MAME region "%s"' % tag) if tag else ', unused',
                 (' (window = MAME region %s)' % ', '.join('0x%X-0x%X' % (ro, ro + n - 1) for wo, ro, n in xf)) if xf else ''))
        for item in window_plan(e, reg):
            if item[0] == 'fill':
                L.append('        <part repeat="0x%X">%02X</part>' % (item[1], item[2]))
            elif item[0] == 'load':
                ld = item[1]
                if ld['kind'] == 'ROM_LOAD16_WORD_SWAP':
                    L.append('        <!-- ROM_LOAD16_WORD_SWAP: the bytes of every 16-bit word exchanged -->')
                    L.append('        <interleave output="16">')
                    L.append('            <part name="%s" crc="%s" map="12"/>' % (ld['name'], ld['crc']))
                    L.append('        </interleave>')
                else:
                    L.append('        <part name="%s" crc="%s"/>' % (ld['name'], ld['crc']))
            else:
                a0, a1 = item[1]
                L.append('        <!-- ROM_LOAD32_WORD: %s -> bytes 0-1, %s -> bytes 2-3 of each longword -->'
                         % (a0['name'], a1['name']))
                L.append('        <interleave output="32">')
                L.append('            <part name="%s" crc="%s" map="0021"/>' % (a0['name'], a0['crc']))
                L.append('            <part name="%s" crc="%s" map="2100"/>' % (a1['name'], a1['crc']))
                L.append('        </interleave>')
    L.append('    </rom>')
    L.append('</misterromdescription>')
    with open(a.out, 'w', newline='\n', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')
    print('wrote', a.out, '(%d check entries)' % len(ents))

# ---------------------------------------------------------------------------
# MiSTer mra_loader.cpp emulation (Main_MiSTer support/arcade/mra_loader.cpp: rom_data(), interleave,
# <part repeat>, hex parts), as in the NB-1 tool.
class MisterRom:
    def __init__(self):
        self.data = bytearray()
        self.romlen = [0] * 8
        self.unitlen = 1

    def _ensure(self, n):
        if len(self.data) < n:
            self.data += bytes(max(n - len(self.data), 1 << 20))

    def rom_data(self, buf, imap):
        m = imap or 1
        idx, mr = 0, m
        for _ in range(self.unitlen):
            if mr & 0xF:
                break
            mr >>= 4
            idx += 1
        if idx >= self.unitlen:
            fail('illegal map')
        offsets, first, gaps, mr = [], True, 0, m
        for _ in range(self.unitlen):
            if mr & 0xF:
                offsets.append(idx + (mr & 0xF) - 1 + gaps)
                first = False
            elif not first:
                gaps += 1
            mr >>= 4
        if self.unitlen == 1 and offsets == [0]:
            self._ensure(self.romlen[idx] + len(buf))
            self.data[self.romlen[idx]: self.romlen[idx] + len(buf)] = buf
            self.romlen[idx] += len(buf)
            return
        k = len(offsets)
        n = len(buf) // k
        self._ensure(self.romlen[idx] + n * self.unitlen)
        base = self.romlen[idx]
        for j, off in enumerate(offsets):
            self.data[base + off: base + n * self.unitlen: self.unitlen] = buf[j::k][:n]
        self.romlen[idx] += n * self.unitlen

def mister_stream(mra_path, rom_index, files):
    root = ET.parse(mra_path).getroot()
    roms = [r for r in root.findall('rom') if int(r.get('index', '0')) == rom_index]
    if len(roms) != 1:
        fail('expected one <rom index=%d>' % rom_index)
    st = MisterRom()

    def part(p, imap):
        rep = int(p.get('repeat', '1'), 0)
        if p.get('name'):
            if p.get('offset') or p.get('length'):
                fail('part offset/length not used by NB-2 MRAs')
            d = files[p.get('name')]
        else:
            d = bytes.fromhex(''.join((p.text or '').split()))
        if len(d) == 1 and rep > 1 and imap == 0 and st.unitlen == 1:
            st.rom_data(d * rep, 0)
        else:
            for _ in range(rep):
                st.rom_data(d, imap)

    for node in roms[0]:
        if node.tag == 'part':
            st.unitlen = 1
            part(node, int(node.get('map', '0'), 16))
        elif node.tag == 'interleave':
            if int(node.get('input', '8')) != 8:
                fail('interleave input must be 8')
            st.unitlen = int(node.get('output')) // 8
            for i in range(1, 8):
                st.romlen[i] = st.romlen[0]
            for p in node.findall('part'):
                part(p, int(p.get('map', '0'), 16))
            st.unitlen = 1
        elif node.tag is ET.Comment:
            pass
    return bytes(st.data[:st.romlen[0]]), roms[0]

# ---------------------------------------------------------------------------
# validate
def synthetic_files(game):
    files = {}
    for reg in game['regions']:
        for ld in reg['loads']:
            if ld['name'] in files:
                continue
            seed = hashlib.sha256(ld['name'].encode()).digest()
            n = ld['length']
            blk = bytearray()
            ctr = 0
            while len(blk) < n:
                blk += hashlib.sha256(seed + ctr.to_bytes(8, 'little')).digest()
                ctr += 1
            files[ld['name']] = bytes(blk[:n])
    return files

def pkg_map():
    src = open(os.path.join(REPO, 'rtl', 'nb2', 'nb2_mem_pkg.sv')).read()
    base = dict(re.findall(r"REG_(\w+):\s*region_base\s*=\s*26'h([0-9A-Fa-f]+);", src))
    size = dict(re.findall(r"REG_(\w+):\s*region_size\s*=\s*26'h([0-9A-Fa-f]+);", src))
    ids = dict(re.findall(r"localparam logic \[3:0\] REG_(\w+)\s*=\s*4'd(\d+);", src))
    end = re.search(r"STREAM_END\s*=\s*27'h([0-9A-Fa-f]+)", src).group(1)
    return {k: (int(ids[k]), int(base[k], 16), int(size[k], 16)) for k in base}, int(end, 16)

def load_zip_files(zpaths, meta):
    files = {}
    zs = [zipfile.ZipFile(zp) for zp in zpaths.split('|')]
    for name, ld in meta.items():
        info, z = None, None
        for zz in zs:
            infos = zz.infolist()
            info = {'%08x' % i.CRC: i for i in infos}.get(ld['crc']) or {i.filename: i for i in infos}.get(name)
            if info is not None:
                z = zz
                break
        if info is None:
            fail('%s not in %s' % (name, zpaths))
        d = z.read(info)
        if len(d) != ld['length'] or '%08x' % crc32(d) != ld['crc'] or hashlib.sha1(d).hexdigest() != ld['sha1']:
            fail('%s: size/CRC/SHA1 mismatch in the zip' % name)
        files[name] = d
    return files

def cmd_validate(a):
    game = json.load(open(a.game))
    checks = 0
    pm, pend = pkg_map()
    for name, rid, base, size, tag, xf in PLATFORM_MAP:
        if tag is None:
            continue
        if pm.get(name) != (rid, base, size):
            fail('map entry %s differs from nb2_mem_pkg.sv: %s' % (name, pm.get(name)))
        checks += 1
    if pend != STREAM_END:
        fail('STREAM_END differs from nb2_mem_pkg.sv')
    if a.listxml:
        root = ET.parse(a.listxml).getroot()
        mach = {m.get('name'): m for m in root.iter('machine')}
        known = {}
        for m in (game['set'], 'namcoc75'):
            for r in mach[m].findall('rom'):
                known[r.get('name')] = (int(r.get('size')), r.get('crc'), r.get('sha1'))
        for reg in game['regions']:
            for ld in reg['loads']:
                if ld.get('reload'):
                    continue
                if known.get(ld['name']) != (ld['length'], ld['crc'], ld['sha1']):
                    fail('JSON metadata differs from listxml for ' + ld['name'])
                checks += 1
        print('  metadata matches -listxml')
    root = ET.parse(a.mra).getroot()
    if root.findtext('rbf') != RBF or root.findtext('setname') != game['set']:
        fail('rbf/setname')
    meta = {ld['name']: ld for r in game['regions'] for ld in r['loads']}
    rom0 = [r for r in root.findall('rom') if r.get('index') == str(IOCTL_ROM)][0]
    used = set()
    for p in rom0.iter('part'):
        if p.get('name'):
            ld = meta.get(p.get('name'))
            if ld is None or p.get('crc') != ld['crc']:
                fail('MRA part %s not in MAME metadata / wrong CRC' % p.get('name'))
            used.add(p.get('name'))
    if used != set(meta):
        fail('MRA does not load exactly the MAME files: missing %s' % sorted(set(meta) - used))
    checks += len(used)
    syn = synthetic_files(game)
    got, _ = mister_stream(a.mra, IOCTL_ROM, syn)
    exp = platform_stream(game, syn)
    if len(got) != len(exp):
        fail('stream length %X, expected %X' % (len(got), len(exp)))
    if got != exp:
        i = next(i for i in range(len(exp)) if got[i] != exp[i])
        fail('stream differs from MAME layout at 0x%X' % i)
    checks += 1
    print('  MiSTer-emulated index-0 stream == MAME region layout in the NB-2 map (0x%X bytes)' % len(got))
    recb, _ = mister_stream(a.mra, IOCTL_CHECK, {})
    slen, ents = parse_record(recb)
    if slen != STREAM_END:
        fail('record stream length')
    if ents != check_entries(game):
        fail('check record differs from the entries derived from MAME metadata')
    idmap = {rid: (name, base, size) for name, rid, base, size, tag, xf in PLATFORM_MAP if tag}
    cover = {rid: [] for rid in idmap}
    for rid, lanes, o, n, crcs in ents:
        cover[rid].append((o, n))
        if lanes not in (1, 2, 4, LANES_SWAP16) or o % 4 or n % 4:
            fail('entry shape')
    for rid, spans in cover.items():
        pos = 0
        for o, n in sorted(spans):
            if o != pos:
                fail('check record gap/overlap in %s at 0x%X' % (idmap[rid][0], pos))
            pos = o + n
        if pos != idmap[rid][2]:
            fail('check record does not cover all of ' + idmap[rid][0])
        checks += 1
    # synthetic: each entry's CRCs computed on the stream equal the CRCs of the source files
    for rid, lanes, o, n, crcs in ents:
        got_c = entry_crcs(exp, idmap[rid][1], o, n, lanes)
        if [c for c in got_c] != [c for c in synth_entry_crcs(game, syn, rid, lanes, o, n)]:
            fail('entry %s+0x%X: lane assignment' % (idmap[rid][0], o))
    print('  check record: %d entries, cover every checked byte once, CRCs = MAME file CRCs / fill CRCs' % len(ents))
    brd, _ = mister_stream(a.mra, IOCTL_BOARD, {})
    if brd != board_bytes(game):
        fail('board record differs from the JSON board description')
    if a.mame_src:
        ref = board_from_source(open(a.mame_src, encoding='utf-8').read(), game['set'])
        kc = game['board']['keycus']
        if any(ref['keycus'][k] != kc[k] for k in ('mode', 'id', 'id_word', 'rnd_word', 'part')) \
           or ref['flags'] != game['board']['flags']:
            fail('JSON board description differs from MAME: %r' % ref)
        checks += 1
    checks += 1
    print('  board record: KEYCUS mode %d ID $%04X, flags $%02X, %d hot pages%s' % (
        game['board']['keycus']['mode'], game['board']['keycus']['id'], game['board']['flags'],
        len(game['board']['hot_pages']), ' (= MAME)' if a.mame_src else ''))
    nv = root.findall('nvram')
    if len(nv) != 1 or nv[0].get('index') != str(IOCTL_NVRAM) or int(nv[0].get('size'), 0) != NVRAM_BYTES:
        fail('<nvram> missing or wrong')
    eimg, _ = mister_stream(a.mra, IOCTL_NVRAM, {})
    if eimg != bytes([0xFF]) * NVRAM_BYTES:
        fail('EEPROM power-on image is not erased')
    kids = list(root)
    rom1 = [r for r in root.findall('rom') if r.get('index') == str(IOCTL_NVRAM)][0]
    if not (kids.index(rom1) < kids.index(nv[0]) < kids.index(rom0)):
        fail('order must be: EEPROM image, <nvram>, then the ROM stream')
    checks += 2
    files = None
    if a.zip:
        files = load_zip_files(a.zip, meta)
        real, _ = mister_stream(a.mra, IOCTL_ROM, files)
        if real != platform_stream(game, files):
            fail('real stream differs from MAME layout')
        for rid, lanes, o, n, crcs in ents:
            if entry_crcs(real, idmap[rid][1], o, n, lanes) != crcs:
                fail('check entry %s+0x%X fails on the real ROM set' % (idmap[rid][0], o))
        p = idmap[0][1]
        ssp, pc = struct.unpack_from('>II', real, p)
        print('  local ROM set: all %d files match MAME SHA1; all %d check entries pass on the real stream'
              % (len(files), len(ents)))
        print('  reset vectors: SSP $%08X, PC $%08X' % (ssp, pc))
        checks += len(ents) + 1
        if a.mame_regions:
            regs = {r['tag']: r for r in game['regions']}
            for e in PLATFORM_MAP:
                name, rid, base, size, tag, xf = e
                if tag is None:
                    continue
                dump = os.path.join(a.mame_regions, tag.replace(':', '_') + '.bin')
                img = open(dump, 'rb').read()
                if real[base:base+size] != window_bytes(e, img):
                    w = window_bytes(e, img)
                    i = next(i for i in range(size) if real[base+i] != w[i])
                    fail('%s window differs from MAME region memory at +0x%X' % (name, i))
                # bytes of the MAME region not in the window must be fill or mirrors of window bytes
                checks += 1
            print('  every window of the real stream == MAME region memory (scripts/mame/nb2_regions.lua dumps)')
    print('PASS MRA VALIDATION: %s, %d checks' % (os.path.basename(a.mra), checks))

def synth_entry_crcs(game, syn, rid, lanes, o, n):
    """Independent route: the CRCs of the files (or fill) the entry stands for."""
    regs = {r['tag']: r for r in game['regions']}
    e = [x for x in PLATFORM_MAP if x[1] == rid][0]
    reg = regs.get(e[4])
    pos = 0
    for item in window_plan(e, reg):
        if item[0] == 'fill':
            if pos == o:
                return [crc32(bytes([item[2]]) * item[1])]
            pos += item[1]
        elif item[0] == 'load':
            if pos == o:
                return [crc32(syn[item[1]['name']])]
            pos += item[1]['length']
        else:
            if pos == o:
                return [crc32(syn[g['name']]) for g in item[1]]
            pos += 2 * item[1][0]['length']
    fail('no plan item at 0x%X' % o)

# ---------------------------------------------------------------------------
# dump: the ioctl streams the MRA sends (indices 3, 2, 0) as binary files, for the RTL loader bench.
# ROM-derived output: write it outside the repository.
def cmd_dump(a):
    game = json.load(open(a.game))
    meta = {ld['name']: ld for r in game['regions'] for ld in r['loads']}
    files = load_zip_files(a.zip, meta)
    os.makedirs(a.out_dir, exist_ok=True)
    for idx, name in ((IOCTL_CHECK, 'index3.bin'), (IOCTL_BOARD, 'index2.bin'), (IOCTL_ROM, 'index0.bin')):
        data, _ = mister_stream(a.mra, idx, files if idx == IOCTL_ROM else {})
        open(os.path.join(a.out_dir, name), 'wb').write(data)
        print('wrote %s (%d bytes)' % (os.path.join(a.out_dir, name), len(data)))

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sp = ap.add_subparsers(dest='cmd', required=True)
    e = sp.add_parser('extract')
    e.add_argument('--mame-src', required=True)
    e.add_argument('--listxml', required=True)
    e.add_argument('--set', required=True)
    e.add_argument('--hot-pages', required=True, help='4 KiB page numbers (hex, e.g. "208 209 240")')
    e.add_argument('--hot-pages-note')
    e.add_argument('--note')
    e.add_argument('-o', '--out', required=True)
    g = sp.add_parser('generate')
    g.add_argument('--game', required=True)
    g.add_argument('-o', '--out', required=True)
    v = sp.add_parser('validate')
    v.add_argument('--mra', required=True)
    v.add_argument('--game', required=True)
    v.add_argument('--listxml')
    v.add_argument('--zip', help="the local ROM set; several zips separated by '|' (set|parent|namcoc75)")
    v.add_argument('--mame-regions', help='directory of MAME region dumps (scripts/mame/nb2_regions.lua), with --zip')
    v.add_argument('--mame-src')
    d = sp.add_parser('dump')
    d.add_argument('--mra', required=True)
    d.add_argument('--game', required=True)
    d.add_argument('--zip', required=True)
    d.add_argument('-o', '--out-dir', required=True)
    a = ap.parse_args()
    {'extract': cmd_extract, 'generate': cmd_generate, 'validate': cmd_validate, 'dump': cmd_dump}[a.cmd](a)

if __name__ == '__main__':
    main()
