#!/usr/bin/env python3
"""Pre-push gate for the boot-progress archive, runnable on any host without the XNU source tree.

A package run costs about an hour, so the cheap checks that would have caught the two 27 link
failures run here first: the Font.h search path, the profile macros, and a real preprocessing pass
proving that no virtio symbol survives into the 26/27 build. Needs pcpp (pip install pcpp) for the
preprocessing pass; the remaining checks run without it.
"""
import io
import os
import re
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROGRESS = ROOT / 'package/BootProgress.c'
BUILDER = ROOT / 'scripts/build-boot-progress.py'
FONT_HEADER = Path('Silicon/Silicium/SiliciumPkg/Library/FrameBufferSerialPortLib')
VIRTIO = ('native_virtio_ready', 'native_virtio_read_requests',
          'native_virtio_read_failures', 'native_virtio_verified_blocks')
PROFILES = {'sheng_defconfig': (True, True),              # 17: boot page, links the virtio driver
            'sheng_ipados26_defconfig': (False, False),   # 26: no page, no driver
            'sheng_ipados27_defconfig': (False, False)}   # 27: page off for the __DATA baseline run
failures = []


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        failures.append(name)


def expand(defines):
    """Preprocess BootProgress.c with the given macros. Includes are dropped: this host has no XNU tree."""
    import pcpp
    pre = pcpp.Preprocessor()
    for key, value in defines.items():
        pre.define(f'{key} {value}')
    text = '\n'.join(line for line in PROGRESS.read_text(encoding='utf-8').splitlines()
                     if not line.strip().startswith('#include'))
    pre.parse(text)
    out = io.StringIO()
    pre.write(out)
    return out.getvalue()


def symbols_in(text):
    return [name for name in VIRTIO if re.search(r'(?<!\w)' + name + r'(?!\w)', text)]


print('=== headers ===')
for header in ('BootProgressCore.h', 'BootProgressDisplay.h',
               'BootProgressPulse.h', 'BootProgressLifecycle.h'):
    check(header, (ROOT / 'package' / header).is_file())
check('Font.h resolved by the builder', 'FONT_HEADER' in BUILDER.read_text(encoding='utf-8'))

print('\n=== symbols (real preprocessing) ===')
try:
    plain = expand({})
    virtio = expand({'XNU_NATIVE_VIRTIO_BLOCK': '1'})
except ImportError:
    print('  [SKIP] pcpp is not installed (pip install pcpp)')
else:
    # 27 and 26 compile this file without the driver: no virtio name may reach the object at all,
    # because a constant condition still emits a relocation for every symbol it names.
    check('26/27: no virtio symbol emitted', not symbols_in(plain), f'{symbols_in(plain) or "none"}')
    check('17: all four retained', symbols_in(virtio) == list(VIRTIO), f'{len(symbols_in(virtio))}/4')

print('\n=== builder forwards the personality macro ===')
builder = BUILDER.read_text(encoding='utf-8')
check('XNU_NATIVE_VIRTIO_BLOCK passed to the compile',
      'XNU_NATIVE_VIRTIO_BLOCK' in builder and 'command.append' in builder)

print('\n=== porting.toml ===')
porting = (ROOT / 'porting.toml').read_text(encoding='utf-8')
check('weak_import limitation recorded', 'weak_import is not enough' in porting)
check('parses', bool(tomllib.loads(porting)))

print('\n=== profile macros ===')
for name, (page, driver) in PROFILES.items():
    text = (ROOT / 'configs' / name).read_text(encoding='utf-8')
    # A commented-out setting must read as off, so match only uncommented lines.
    enabled = {line.strip() for line in text.splitlines()
               if line.strip() and not line.lstrip().startswith('#')}
    got = ('CONFIG_XNU_BOOT_PROGRESS=y' in enabled, 'CONFIG_XNU_NATIVE_VIRTIO_BLOCK=y' in enabled)
    check(name, got == (page, driver), f'page={got[0]} driver={got[1]}')

print('\n' + ('ALL PASS' if not failures else f'FAILED: {failures}'))
sys.exit(1 if failures else 0)
