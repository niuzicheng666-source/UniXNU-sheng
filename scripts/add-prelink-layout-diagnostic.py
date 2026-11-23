#!/usr/bin/env python3
"""Print the byte accounting behind a prelink segment-overlap failure.

The 27 prelink stops in fileset_layout.validate() with "Native __DATA overlaps a
retained segment". That message names neither the ranges nor the excess, and the
only place the Apple kernelcache this needs exists is a CI runner, so the numbers
have to come out of the run that fails. This patches the XnuPrelink checkout the
workflow just synced (UniXNU/kits/XnuPrelink) to report them before raising, and
leaves the failure itself unchanged.

The patch is idempotent and only touches a checkout, never a repository.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

ORIGINAL = """            if any(begin < e and b < end for b, e in self.occupied + placed):
                raise ValueError(f'Native {segment["name"]} overlaps a retained segment')"""

PATCHED = """            overlap = [(b, e) for b, e in self.occupied + placed if begin < e and b < end]
            if overlap:
                # Diagnostic for the 27 prelink stop: the accounting the failure needs.
                root = self.roots.get(segment['name'])
                total = sum(e - b for b, e in overlap)
                print(f'[layout] native {segment["name"]} at {begin:#x}..{end:#x} '
                      f'({end - begin:#x} bytes)', flush=True)
                print(f'[layout] root mapping: {root["va"]:#x}..{root["va"] + root["size"]:#x} '
                      f'({root["size"]:#x} bytes)' if root else '[layout] no root for this segment',
                      flush=True)
                print(f'[layout] overlaps {len(overlap)} retained range(s), {total:#x} bytes',
                      flush=True)
                for b, e in sorted(overlap):
                    print(f'[layout]   retained {b:#x}..{e:#x} ({e - b:#x} bytes), '
                          f'intersects {min(end, e) - max(begin, b):#x} bytes', flush=True)
                for name, address in sorted(self.placements.items(), key=lambda kv: kv[1]):
                    size = next((s['size'] for s in self.native.segments if s['name'] == name), 0)
                    print(f'[layout]   placed {name} {address:#x}..{address + size:#x}', flush=True)
                raise ValueError(f'Native {segment["name"]} overlaps a retained segment')"""


def main():
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'UniXNU/kits/XnuPrelink/fileset_layout.py'
    if not target.is_file():
        # The kits checkout is a submodule the packaging step provisions; skipping it
        # must not fail a build that would otherwise run.
        print(f'layout diagnostic: {target} not present, nothing to patch')
        return 0
    text = target.read_text(encoding='utf-8')
    if 'layout] native' in text:
        print(f'layout diagnostic: {target} already carries it')
        return 0
    if ORIGINAL not in text:
        print(f'layout diagnostic: the validated block is not in {target}; '
              'XnuPrelink changed, review the diagnostic', file=sys.stderr)
        return 1
    target.write_text(text.replace(ORIGINAL, PATCHED, 1), encoding='utf-8')
    print(f'layout diagnostic: patched {target}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
