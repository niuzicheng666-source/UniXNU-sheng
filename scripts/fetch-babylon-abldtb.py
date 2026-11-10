#!/usr/bin/env python3
"""Give bootimg the DTB the bootloader actually applies its overlays to.

Why (evidence from the device's own log ring, /dev/logfs/UefiLog1.txt):

    Handling Cmd: boot
    Booting from slot (_a)
    Decompressing kernel image total time: 197 ms
    ApplyOverlay: ufdt apply overlay failed
    Error: Dtb overlay failed
    reserved-memory node not found in device tree
    Device Tree update failed Status:Not Found
    Handling Cmd: getvar:all          <-- back in fastboot, never jumped

ABL merges the entries of dtbo_a into the device tree appended to a v0 boot
image, and libufdt resolves each overlay's __fixups__ labels through the base
tree's /__symbols__. A dtc build without -@ carries no /__symbols__ node at all,
so every fixup fails, the overlay is rejected, and ABL returns to fastboot
instead of entering the payload. The MI logo on screen during that is ABL's own
unlock logo, drawn a few lines earlier.

vendor_boot.img ships the base trees the overlays were generated against. The
entry whose qcom,msm-id is 0x207/0x20000 -- the value this handset reports as
hw-revision 20000 -- carries the 1538 __symbols__ labels the babylon overlays
expect. Copy that entry out as build/babylon/babylon.dtb.

Only the bootloader reads this file: the kernel's tree is
device-tree/hardware.dtb, built from dts/babylon.dts by the payload step.
"""
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / '.cache/firmware/sheng/porting-inputs'
OUTPUT = Path('build/babylon/babylon.dtb')
BOOT_SOURCES = ('vendor_boot.img', 'vendor_boot_a.img')
OVERLAY_SOURCES = ('dtbo.img', 'dtbo_a.img')

FDT_MAGIC = b'\xd0\x0d\xfe\xed'
# The stock tree listed for this SoC generation: msm-id 0x207 with the v2.0
# revision word. It is also the one every babylon dtbo entry targets.
EXPECTED_MSM_ID = (0x207, 0x20000)
MIN_SYMBOLS = 1000


def walk(blob):
    """Parse one standalone FDT into {path: {prop: bytes}}."""
    structure = struct.unpack_from('>I', blob, 8)[0]
    strings = struct.unpack_from('>I', blob, 12)[0]
    strings_bytes = struct.unpack_from('>I', blob, 32)[0]
    structure_bytes = struct.unpack_from('>I', blob, 36)[0]
    cursor, limit = structure, structure + structure_bytes
    stack, props = [], {}
    while cursor + 4 <= limit:
        tag = struct.unpack_from('>I', blob, cursor)[0]
        cursor += 4
        if tag == 1:
            end = blob.find(b'\0', cursor, limit)
            if end < 0:
                break
            stack.append(blob[cursor:end].decode('utf-8', 'replace'))
            cursor = (end + 4) & ~3
        elif tag == 2:
            if stack:
                stack.pop()
        elif tag == 3:
            length, name_offset = struct.unpack_from('>II', blob, cursor)
            cursor += 8
            end = blob.find(b'\0', strings + name_offset, strings + strings_bytes)
            if end < 0 or cursor + length > limit:
                break
            name = blob[strings + name_offset:end].decode('utf-8', 'replace')
            path = '/' + '/'.join(stack[1:]) if len(stack) > 1 else '/'
            props.setdefault(path, {})[name] = blob[cursor:cursor + length]
            cursor = (cursor + length + 3) & ~3
        elif tag == 9:
            break
        else:
            cursor = (cursor + 3) & ~3
    return props


def cells(value):
    return [struct.unpack_from('>I', value, k)[0] for k in range(0, len(value) - 3, 4)]


def symbol_labels(props):
    return set(props.get('/__symbols__', {}))


def fixup_labels(props):
    """__fixups__ children are keyed by the label they need."""
    return set(props.get('/__fixups__', {}))


def scan_boot(blob):
    """Yield (offset, total, props) for every DTB inside a boot image."""
    cursor = 0
    while cursor < len(blob) - 40:
        if blob[cursor:cursor + 4] != FDT_MAGIC:
            cursor += 4
            continue
        total = struct.unpack_from('>I', blob, cursor + 4)[0]
        if not 40 < total <= len(blob) - cursor:
            cursor += 4
            continue
        yield cursor, total, walk(blob[cursor:cursor + total])
        cursor += total


def scan_dtbo(blob):
    """Yield (index, props) for the overlay trees inside an AOSP dtbo image."""
    magic, _total, _header, entry_size, count, entries_off, _page, _ver = struct.unpack_from('>8I', blob, 0)
    if magic != 0xd7b7ab1e:
        raise ValueError('dtbo header magic is not 0xd7b7ab1e')
    for index in range(count):
        base = entries_off + index * entry_size
        size, offset, _ident, _rev = struct.unpack_from('>4I', blob, base)
        if size and offset < len(blob) and offset + size <= len(blob):
            yield index, walk(blob[offset:offset + size])


def load(directory, names):
    for name in names:
        path = directory / name
        if path.is_file():
            return path, path.read_bytes()
    return None, None


def main():
    boot_path, boot = load(FIRMWARE, BOOT_SOURCES)
    if boot is None:
        raise SystemExit(f'no stock vendor_boot.img under {FIRMWARE}: '
                         'download-sheng-firmware.py and extract-sheng-firmware.py must run first')

    chosen = None
    available = []
    for offset, total, props in scan_boot(boot):
        root = props.get('/', {})
        msm = cells(root.get('qcom,msm-id', b''))
        labels = len(symbol_labels(props))
        model = root.get('model', b'').rstrip(b'\0').decode('utf-8', 'replace')
        available.append((offset, total, msm[:2], labels, model))
        if tuple(msm[:2]) == EXPECTED_MSM_ID and labels >= MIN_SYMBOLS:
            chosen = (offset, total, props)

    if chosen is None:
        for offset, total, msm, labels, model in available:
            print(f'  candidate @{offset:#x}: msm={[hex(v) for v in msm]} symbols={labels} {model}')
        raise SystemExit(f'{boot_path.name}: no DTB with msm-id '
                         f'{EXPECTED_MSM_ID[0]:#x}/{EXPECTED_MSM_ID[1]:#x} and {MIN_SYMBOLS}+ symbols')

    offset, total, props = chosen
    model = props['/'].get('model', b'').rstrip(b'\0').decode('utf-8', 'replace')
    labels = symbol_labels(props)
    if '/reserved-memory' not in props:
        raise SystemExit('chosen DTB has no /reserved-memory: the splash node patch would fail')

    # Self-check against the overlays this device really applies. The babylon
    # entries are the ones carrying qcom,board-id 0x10008.
    dtbo_path, dtbo = load(FIRMWARE, OVERLAY_SOURCES)
    if dtbo is not None:
        needed, seen = set(), 0
        for index, overlay in scan_dtbo(dtbo):
            root = overlay.get('/', {})
            if cells(root.get('qcom,board-id', b''))[:2] != [0x10008, 0x0]:
                continue
            seen += 1
            needed |= fixup_labels(overlay)
        missing = sorted(needed - labels)
        print(f'overlays: {seen} babylon entries reference {len(needed)} labels; '
              f'{len(needed) - len(missing)} resolve against this tree', flush=True)
        if missing:
            raise SystemExit('chosen DTB still misses labels: ' + ', '.join(missing[:12]))
    else:
        print(f'overlays: {"/".join(OVERLAY_SOURCES)} not extracted; skipping the cross-check', flush=True)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_bytes(boot[offset:offset + total])
    print(f'ABL device tree: {OUTPUT} <- {boot_path.name} @{offset:#x} '
          f'({total} bytes, {len(labels)} symbols, model={model!r})', flush=True)


if __name__ == '__main__':
    main()
