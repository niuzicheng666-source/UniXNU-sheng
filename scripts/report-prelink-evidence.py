#!/usr/bin/env python3
"""Print the three numbers a prelink __DATA overflow cannot be planned without.

The 27 prelink stops in fileset_layout.validate() because the native __DATA of the
kernel binary is larger than the room before the first retained kext segment. The
dump step shows the last lines of each log, and the two files that settle it are not
logs at all, so they never reached a report:

  1. What LD_KERNEL_ARCHIVES actually expanded to. XNU writes it to
     <objroot>/<kernel>.link/<kernel>.linkarchives while linking, so this says
     whether an archive such as libBootProgress.a was passed at all. The build
     script's own `set -x` line quotes the unexpanded form and cannot answer it.

  2. The size of each section in the kernel binary, so the 3 MiB __DATA can be
     attributed. Without it any plan to shrink it is a guess. `size -m` is used
     when present; the Mach-O header is parsed directly when it is not, so this
     number cannot go missing on a runner that lacks the tool.

  3. build/boot-progress/build-report.json and segments.txt, which
     build-boot-progress.py already fills with the archive digest and, once the
     build prints its segments, the per-member segment sizes.

Read-only: nothing is written and no build input changes. Run it after a failed
step as well as a successful one, since a passing build still needs 1 and 2.
Every section is guarded: evidence that is missing must be reported as missing,
never crash the step and take the rest of the report with it.
"""
import json
import struct
import subprocess
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# CI checks this repository out as `device` with UniXNU beside it, not inside it; a local
# checkout may have it either way, so both are tried.
OUT = next((p for p in (ROOT.parent / 'UniXNU' / 'out', ROOT / 'UniXNU' / 'out') if p.is_dir()),
           ROOT.parent / 'UniXNU' / 'out')

MH_MAGIC_64 = 0xfeedfacf
LC_SEGMENT_64 = 0x19


def report(heading):
    print(f'\n===== {heading} =====', flush=True)


def guarded(section):
    """Run one section; a failure names itself and the rest of the report still prints."""
    try:
        section()
    except Exception:                                    # noqa: BLE001 - evidence must not abort the step
        print(f'  (this section failed; the rest of the report follows)')
        for line in traceback.format_exc().rstrip().splitlines():
            print(f'  {line}')


def find_kernel_links():
    """<objroot>/<kernel>.link/ directories, wherever this profile put its objroot.

    The directory is named after the kernel it links, so the stem is read back from the
    file inside it rather than assumed: a checkout may carry several builds.
    """
    if not OUT.is_dir():
        return []
    return sorted({p.parent for p in OUT.glob('**/*.link/*.linkarchives')})


def show_links():
    report('LD_KERNEL_ARCHIVES as XNU received it')
    links = find_kernel_links()
    if not links:
        print('no <kernel>.link/ directory under UniXNU/out - the kernel never reached the '
              'link step, so there is nothing to report here')
        return
    for link in links:
        print(f'--- {link}')
        # The files are <kernel>.linkarchives beside <kernel>.link/, named after the same stem.
        stem = next((p.name[:-len('.linkarchives')]
                     for p in link.glob('*.linkarchives')), link.name)
        for name in ('linkarchives', 'linkarguments'):
            path = link / f'{stem}.{name}'
            if not path.is_file():
                print(f'  {name}: absent at {path.name}')
                continue
            text = path.read_text(errors='replace').strip()
            if name == 'linkarchives':
                # One giant line; break before each -Wl, so every input stays whole.
                marked = text.replace(' -Wl,', '\n  -Wl,').replace(' -', '\n  -')
                print(f'  {name} ({len(text.split())} word(s)):')
                for line in marked.splitlines():
                    if line.strip():
                        print(f'    {line.strip()}')
            else:
                print(f'  {name}: {text[:1500]}')
        archives = link / f'{stem}.linkarchives'
        if archives.is_file():
            text = archives.read_text(errors='replace')
            for probe in ('libBootProgress.a', 'BootProgress.o', 'libShengSourceStorage.a'):
                print(f'  contains {probe}: {"yes" if probe in text else "no"}')


def macho_sections(path):
    """(segment, section, size) for a 64-bit Mach-O, read straight from the header.

    Used when no `size` tool is available: the __DATA attribution is the whole point of
    this report, so it must not depend on a binary that may be absent on the runner.
    """
    data = Path(path).read_bytes()
    magic, _, _, _, ncmds, _, _, _ = struct.unpack_from('<IiiIIIII', data, 0)
    if magic != MH_MAGIC_64:
        raise ValueError(f'magic {magic:#x}, not a 64-bit Mach-O')
    offset, rows = 32, []
    for _ in range(ncmds):
        cmd, cmdsize = struct.unpack_from('<II', data, offset)
        if cmd == LC_SEGMENT_64:
            (segname, _, vmsize, _, _, _, _, nsects, _) = \
                struct.unpack_from('<16sQQQQiiII', data, offset + 8)
            segment = segname.rstrip(b'\0').decode('ascii', 'replace')
            rows.append((segment, '(segment)', vmsize))
            for i in range(nsects):
                base = offset + 72 + i * 80
                sectname, _, _, size = struct.unpack_from('<16s16sQQ', data, base)
                rows.append((segment, sectname.rstrip(b'\0').decode('ascii', 'replace'), size))
        offset += cmdsize
    return rows


def show_segments():
    report('kernel binary sections')
    # Only Mach-O images: the .link/ siblings share the prefix but are text files.
    binaries = [p for p in sorted(OUT.glob('**/mach.development.bcm2837*'))
                if p.is_file() and not p.name.endswith(('.dSYM', '.linkarchives', '.linkarguments'))]
    if not binaries:
        print('no mach.development.bcm2837* image under UniXNU/out')
        return
    # The unstripped noctf image is the one emit_prelink.py is given as --kernel.
    preferred = [p for p in binaries if p.name.endswith('.unstripped.noctf')] or binaries
    for path in preferred[:2]:
        print(f'--- {path.name} ({path.stat().st_size:,} bytes on disk)')
        sized = False
        for tool in ('size', 'llvm-size'):
            try:
                result = subprocess.run([tool, '-m', str(path)], capture_output=True,
                                        text=True, timeout=120)
            except (OSError, subprocess.SubprocessError):
                continue
            if result.returncode != 0:
                print(f'  {tool} -m failed (exit {result.returncode}): '
                      f'{result.stderr.strip()[:300]}')
                continue
            shown = 0
            for line in result.stdout.splitlines():
                if line.strip().startswith(('Segment __', 'Section __', 'Total')):
                    print(f'  {line.rstrip()}')
                    shown += 1
            if shown:
                sized = True
                break
            print(f'  {tool} -m produced no segment lines')
        if not sized:
            print('  no size tool gave a table; parsing the Mach-O header instead')
            for segment, sect, size in macho_sections(path):
                label = segment if sect == '(segment)' else f'{segment},{sect}'
                print(f'  {label:<44} {size:>12,} bytes')
        else:
            # The section table is what attributes the 3 MiB; print it whenever __DATA is large.
            rows = [(s, n, z) for s, n, z in macho_sections(path)
                    if s in ('__DATA', '__DATA_CONST', '__BSS') and n != '(segment)']
            if rows:
                print('  __DATA sections (largest first):')
                for segment, sect, size in sorted(rows, key=lambda r: -r[2])[:20]:
                    print(f'    {segment},{sect:<32} {size:>12,} bytes')


def show_boot_report():
    report('build/boot-progress/build-report.json')
    reports = sorted(OUT.glob('**/build/boot-progress/build-report.json'))
    if not reports:
        print('no build/boot-progress/build-report.json under UniXNU/out - '
              'build-boot-progress.py did not run for this profile')
        return
    for path in reports:
        print(f'--- {path}')
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError) as error:
            print(f'  unreadable: {error}')
            data = {}
        for key in sorted(data):
            value = data[key]
            if isinstance(value, str) and len(value) == 64:
                value = value[:16] + '...'
            print(f'  {key}: {value}')
        # segments.txt is written beside the report and survives whichever log carried it.
        segments = path.parent / 'segments.txt'
        if segments.is_file():
            print('  segments.txt:')
            for line in segments.read_text(errors='replace').splitlines():
                print(f'    {line}')
        else:
            print('  segments.txt: absent (build-boot-progress.py did not reach the size step)')


def show_largest_symbols():
    """Name the symbols behind __bss and __common.

    The section table says __DATA is 88% zerofill but not whose zeroes they are, and a plan to
    shrink it needs names, not a percentage: moving a static array out is only possible once we
    know which one. nm -S --size-sort is the cheapest answer and the image is already on disk.
    """
    report('largest __DATA symbols (zerofill first)')
    candidates = [p for p in sorted(OUT.glob('**/mach.development.bcm2837*'))
                  if p.is_file() and not p.name.endswith(('.linkarchives', '.linkarguments'))]
    preferred = [p for p in candidates if p.name.endswith('.unstripped.noctf')] or candidates
    if not preferred:
        print('no kernel image to read symbols from')
        return
    path = preferred[0]
    print(f'--- {path.name}')
    for tool in ('llvm-nm', 'nm'):
        try:
            result = subprocess.run([tool, '-S', '--size-sort', '--numeric-sort', str(path)],
                                    capture_output=True, text=True, timeout=180)
        except (OSError, subprocess.SubprocessError):
            continue
        if result.returncode != 0:
            print(f'  {tool} failed (exit {result.returncode}): {result.stderr.strip()[:200]}')
            continue
        # Rows are: address size type name. Zerofill symbols are 'b'/'B' or 'c'/'C'/'s'/'S'.
        rows = []
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                size = int(parts[1], 16)
            except ValueError:
                continue
            rows.append((size, parts[2], parts[3]))
        if not rows:
            print(f'  {tool} produced no sized symbols')
            continue
        zero = [r for r in rows if r[1] in 'bBcCsS']
        for label, group in (('zerofill (__bss/__common)', zero), ('all symbols', rows)):
            print(f'  {label}: top 15 of {len(group)}')
            for size, kind, name in sorted(group, reverse=True)[:15]:
                print(f'    {size:>12,}  {kind}  {name}')
            if label.startswith('zerofill'):
                print(f'  zerofill total: {sum(r[0] for r in zero):,} bytes')
        return
    print('  no nm tool available; symbol attribution skipped')


def show_layout_diagnostic():
    report('prelink layout accounting ([layout] lines in any log)')
    logs = sorted(OUT.glob('**/logs/prelink.log'))
    if not logs:
        print('no logs/prelink.log under UniXNU/out')
        return
    for path in logs[:2]:
        print(f'--- {path}')
        lines = [l for l in path.read_text(errors='replace').splitlines()
                 if l.startswith('[layout]')]
        for line in lines[:120]:
            print(f'  {line}')
        if len(lines) > 120:
            print(f'  ... {len(lines) - 120} more [layout] line(s)')
        if not lines:
            print('  no [layout] lines: add-prelink-layout-diagnostic.py did not patch '
                  'fileset_layout.py, or validate() did not reach the overlap')


def main():
    print(f'prelink evidence: root {ROOT}')
    if not OUT.is_dir():
        print(f'prelink evidence: {OUT} is absent; nothing to report')
        return 0
    guarded(show_links)
    guarded(show_segments)
    guarded(show_largest_symbols)
    guarded(show_boot_report)
    guarded(show_layout_diagnostic)
    report('end of prelink evidence')
    return 0


if __name__ == '__main__':
    sys.exit(main())
