#!/usr/bin/env python3
"""Compile the babylon FDT for the boot image from the sheng mainline DTS closure.

babylon (Xiaomi MIX Fold 3) shares the SM8550 SoC closure with sheng
(sm8550.dtsi and the pm*.dtsi family), so the sheng board file is fetched to
obtain the full include closure, then only the board identity lines are
rewritten and the result is compiled to build/babylon/babylon.dtb.

The boot image's device tree is read by the stock ABL/XBL before it hands off
to UniXNU, so it must carry babylon's own qcom,msm-id / qcom,board-id. The
stock babylon board-id is 0x10008 (same as sheng) and its SM8550 msm-id is
0x207 (Kalama v2), unlike sheng's 0x218.
"""
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import posixpath
import re
import subprocess
import urllib.request

COMMIT = '42f3b40c702abbc37c3a04c95af008cd283aeaf3'
REPOSITORY = 'https://github.com/ianchb/sm8550-mainline'
BASE = f'https://raw.githubusercontent.com/ianchb/sm8550-mainline/{COMMIT}/'
ROOT = Path('.cache/linux-dts')
BOARD = 'arch/arm64/boot/dts/qcom/sm8550-xiaomi-sheng.dts'
BABYLON_BOARD = 'arch/arm64/boot/dts/qcom/sm8550-xiaomi-babylon.dts'
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"\n]+)[>"]', re.M)

# Board identity rewrites applied to the sheng source. Only identifiers change;
# the SM8550 SoC/board structure is shared with sheng.
REWRITES = [
    ('Xiaomi Pad 6S Pro 12.4', 'Xiaomi MIX Fold 3'),
    ('"xiaomi,sheng", "qcom,sm8550"', '"xiaomi,babylon", "qcom,sm8550"'),
    ('chassis-type = "tablet"', 'chassis-type = "handset"'),
    ('<536 0x20000>; /* SM8550P v2.0 */', '<0x207 0x20000>; /* SM8550 v2.0 */'),
    ('Xiaomi-Pad6SPro', 'Xiaomi-MIXFold3'),
]


def fetch(name):
    destination = ROOT / name
    if not destination.exists() and not destination.is_symlink():
        data = urllib.request.urlopen(BASE + name, timeout=30).read()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
    data = (os.readlink(destination).encode() if destination.is_symlink()
            else destination.read_bytes())
    dependencies = set()
    text = data.decode()
    if re.fullmatch(r'(\.\./)+[\w/.,+-]+\s*', text):
        target = posixpath.normpath(str(PurePosixPath(name).parent / text.strip()))
        if target.startswith('../'):
            raise ValueError('Source symlink escapes the source tree')
        if not destination.is_symlink():
            destination.unlink()
            destination.symlink_to(text.strip())
        return name, hashlib.sha256(data).hexdigest(), {target}
    for opening, include in INCLUDE.findall(text):
        if opening == '<':
            dependencies.add('include/' + include)
        else:
            dependencies.add(str(PurePosixPath(name).parent / include))
    return name, hashlib.sha256(data).hexdigest(), dependencies


def fetch_closure():
    pending = {BOARD}
    hashes = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        while pending:
            results = list(executor.map(fetch, sorted(pending)))
            pending = set()
            for name, digest, dependencies in results:
                hashes[name] = digest
                pending |= dependencies
            pending -= hashes.keys()
    (ROOT / 'sources.json').write_text(json.dumps({
        'repository': REPOSITORY, 'commit': COMMIT, 'sha256': hashes,
    }, indent=2) + '\n')
    return hashes


def compile_babylon():
    source = ROOT / BOARD
    text = source.read_text()
    for old, new in REWRITES:
        if old not in text:
            print(f'warning: rewrite target not found: {old!r}')
        text = text.replace(old, new)
    babylon = ROOT / BABYLON_BOARD
    babylon.write_text(text)

    output = Path('build/babylon')
    output.mkdir(parents=True, exist_ok=True)
    preprocessed = output / 'babylon.preprocessed.dts'
    with preprocessed.open('w') as stream:
        subprocess.run(['clang', '-E', '-P', '-nostdinc', '-undef', '-D__DTS__',
                        '-x', 'assembler-with-cpp', '-I', str(ROOT / 'include'),
                        str(babylon)], stdout=stream, check=True)
    with (output / 'dtc.log').open('w') as log:
        subprocess.run(['dtc', '-I', 'dts', '-O', 'dtb', '-o',
                        str(output / 'babylon.dtb'), str(preprocessed)],
                       stderr=log, check=True)
    print(f'Compiled babylon.dtb ({babylon.name})')


if __name__ == '__main__':
    hashes = fetch_closure()
    compile_babylon()
    print(f'fetched {len(hashes)} source files at {COMMIT}')
