#!/usr/bin/env python3
"""Move the UniXNU root manifest off the first block of userdata.

Stock ShengUfs reads the root manifest from the first 4 KiB block of the
"userdata" partition. On a phone that also boots Android that block holds the
f2fs superblock, so a `fastboot boot` image always dies in startRoot with
"root step 3: root manifest rejected" (manifestStatus 1, no SHNGROOT magic).

This script teaches the driver an optional base override:

    findRootLayout (layout, why, baseOverride = 0)
    boot argument: native-ufs-root-base=<absolute 4 KiB LBA on LUN 0>

With the override set, layout->userdataFirst points at the chosen LBA and every
downstream path follows automatically: the manifest read, the window offsets
(gRootObjectBase), the partition-bound check, and the persistent Data store
window (gStoreGeometry). Unset (0) keeps stock behaviour.

Editing the submodule's tracked files is the supported mechanism: UniXNU's
feeds.update_submodules() skips a submodule with local changes ("under local
development; leave its checkout alone"), so this edit survives the package step
instead of being reset to the recorded commit.

usage: apply-shengufs-root-base.py [PATH_TO_ShengUfs]   (default: drivers/ShengUfs, the device submodule)
Idempotent: a second run reports that it is already applied and exits 0.
"""
import sys
from pathlib import Path

DEFAULT = 'drivers/ShengUfs'

LAYOUT_EDITS = [
    # 1. the signature gains the optional base override
    (
        'static bool findRootLayout (RootLayout *layout, const char **why)\n',
        '// baseOverride (absolute 4 KiB LBA on LUN 0) moves the manifest away from the first block\n'
        '// of userdata, so an Android f2fs in front of the root image stays untouched. 0 keeps the\n'
        '// stock behaviour: manifest at the partition start.\n'
        'static bool findRootLayout (RootLayout *layout, const char **why, uint64_t baseOverride = 0)\n',
    ),
    # 2. apply the override after the partition range is known, before the manifest read
    (
        '  gRootDiagnostic.first = layout->userdataFirst;\n'
        '  gRootDiagnostic.last = layout->userdataLast;\n'
        '  gRootDiagnostic.stage = "root manifest";\n',
        '  if (baseOverride) {\n'
        '    if (baseOverride < layout->userdataFirst || baseOverride > layout->userdataLast) {\n'
        '      *why = "root base outside userdata";\n'
        '      return false;\n'
        '    }\n'
        '    layout->userdataFirst = baseOverride;\n'
        '  }\n'
        '  gRootDiagnostic.first = layout->userdataFirst;\n'
        '  gRootDiagnostic.last = layout->userdataLast;\n'
        '  gRootDiagnostic.stage = "root manifest";\n',
    ),
]

HELPER = '''static uint64_t rootBaseOverride (void)
{
  // native-ufs-root-base=<absolute 4 KiB LBA on LUN 0>: where the root manifest lives. Unset
  // (0) means the first block of userdata, the stock layout Android itself formats.
  static uint64_t cached = 0;
  static bool read = false;
  if (!read) {
    uint64_t value = 0;
    if (PE_parse_boot_argn ("native-ufs-root-base", &value, sizeof (value)) && value) cached = value;
    read = true;
  }
  return cached;
}

'''

DRIVER_EDITS = [
    # 3. the helper must exist before NativeUfs::start, whose probe==10 path already calls it
    (
        'bool NativeUfs::start (IOService *provider)\n'
        '{\n'
        '  unsigned probe = 0, rootMode = 0;\n',
        HELPER +
        'bool NativeUfs::start (IOService *provider)\n'
        '{\n'
        '  unsigned probe = 0, rootMode = 0;\n',
    ),
    # 4-6. every call site passes the override
    (
        'if (!layout || !findRootLayout (layout, &why)) {\n'
        '      panic ("NATIVE_UFS root check: %s", why);',
        'if (!layout || !findRootLayout (layout, &why, rootBaseOverride ())) {\n'
        '      panic ("NATIVE_UFS root check: %s", why);',
    ),
    (
        'if (!layout || !findRootLayout (layout, &why)) {\n'
        '      panic ("NATIVE_UFS root windows: %s", why);',
        'if (!layout || !findRootLayout (layout, &why, rootBaseOverride ())) {\n'
        '      panic ("NATIVE_UFS root windows: %s", why);',
    ),
    (
        'if (!layout || !findRootLayout (layout, &why)) {\n'
        '    rootDiagnosticFailure (layout ? why : "root layout allocation failed");',
        'if (!layout || !findRootLayout (layout, &why, rootBaseOverride ())) {\n'
        '    rootDiagnosticFailure (layout ? why : "root layout allocation failed");',
    ),
]


def rewrite(path, edits):
    """Apply ordered replacements once.

    Idempotency has to be tested on the *replacement*, not on the anchor: several
    anchors survive inside their own replacement (the helper sits before the same
    function header, the diagnostic lines are re-emitted verbatim), so an anchor
    test alone would insert the block a second time.
    """
    text = path.read_text(encoding='utf-8')
    applied = 0
    for anchor, replacement in edits:
        if replacement in text:
            continue                        # already applied
        if anchor not in text:
            raise SystemExit(f'{path.name}: anchor missing:\n{anchor[:160]}')
        if text.count(anchor) != 1:
            raise SystemExit(f'{path.name}: anchor appears {text.count(anchor)} times:\n{anchor[:160]}')
        text = text.replace(anchor, replacement, 1)
        applied += 1
    if applied:
        path.write_text(text, encoding='utf-8', newline='\n')
    return applied


def main():
    submodule = Path(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
    if not (submodule / 'RootLayout.h').is_file() or not (submodule / 'NativeUfs.cpp').is_file():
        raise SystemExit(f'{submodule}: not a ShengUfs checkout (RootLayout.h / NativeUfs.cpp missing)')

    layout_changed = rewrite(submodule / 'RootLayout.h', LAYOUT_EDITS)
    driver_changed = rewrite(submodule / 'NativeUfs.cpp', DRIVER_EDITS)

    layout = (submodule / 'RootLayout.h').read_text(encoding='utf-8')
    driver = (submodule / 'NativeUfs.cpp').read_text(encoding='utf-8')

    checks = [
        ('signatures', layout.count('static bool findRootLayout'), 1),
        ('base override blocks', layout.count('if (baseOverride) {'), 1),
        ('helper definitions', driver.count('static uint64_t rootBaseOverride'), 1),
        ('patched call sites', driver.count('findRootLayout (layout, &why, rootBaseOverride ())'), 3),
        ('plain call sites', driver.count('findRootLayout (layout, &why)'), 0),
    ]
    for label, found, expected in checks:
        if found != expected:
            raise SystemExit(f'{submodule}: {label} = {found}, expected {expected}')

    helper_at = driver.find('static uint64_t rootBaseOverride')
    first_use = driver.find('NATIVE_UFS root check')
    if helper_at < 0 or helper_at > first_use:
        raise SystemExit(f'{submodule}: rootBaseOverride is defined after its first use')

    if layout_changed or driver_changed:
        print(f'ShengUfs root base override applied to {submodule} '
              f'(RootLayout.h {layout_changed} edit(s), NativeUfs.cpp {driver_changed} edit(s))')
    else:
        print(f'ShengUfs root base override already present in {submodule}; nothing to do')


if __name__ == '__main__':
    sys.exit(main())
