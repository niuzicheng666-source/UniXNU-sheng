#!/usr/bin/env python3
"""Build the opt-in board diagnostic against this source kernel's actual ABI."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/boot-progress'
OUT.mkdir(parents=True, exist_ok=True)
# BootProgressDisplay.h includes Font.h from the firmware checkout's FrameBufferSerialPortLib, the same header
# xnu-panic-screen.c uses. This script runs as a kernel archive step, not as a firmware preflight script, so the
# unixnu firmware step's UNIXNU_UEFI export is not in the environment: resolve the directory here. Neither the
# release nor CI checks the firmware out beside the build tree (CI passes --uefi ../uefi-sheng from the device
# root, and the tree sits under out/<board>/<profile>/tree), so search the ancestors for the header instead of
# assuming a fixed depth.
FONT_HEADER = Path('Silicon/Silicium/SiliciumPkg/Library/FrameBufferSerialPortLib')
font_dir = Path(os.environ['UNIXNU_UEFI']) / FONT_HEADER if os.environ.get('UNIXNU_UEFI') else None
if font_dir is None or not (font_dir / 'Font.h').is_file():
    candidates = [parent / 'uefi-sheng' / FONT_HEADER for parent in ROOT.parents]
    found = next((c for c in candidates if (c / 'Font.h').is_file()), None)
    if found is None:
        raise SystemExit(f'Font.h not found: set UNIXNU_UEFI to the firmware checkout; searched {[str(c) for c in candidates]}')
    font_dir = found
record = json.loads((ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/osfmk/DEVELOPMENT/startup.o.json').read_text().strip().rstrip(','))
source = ROOT / 'kits/sheng/BootProgress.c'
obj = OUT / 'BootProgress.o'
command = []
args = record['arguments']
i = 0
while i < len(args):
    arg = args[i]
    if arg == record['file']:
        command.append(str(source))
    elif arg in ('-o', '-MF', '-MT', '-MJ'):
        if arg == '-o':
            command += ['-o', str(obj)]
        i += 1
    elif arg == '-Xclang' and args[i+1].startswith('-W'):
        i += 1
    elif arg == '-D' and args[i+1].startswith('XNU_BOOT_PROGRESS='):
        i += 1
    elif not arg.startswith(('-W', '-DXNU_BOOT_PROGRESS=')):
        command.append(arg)
    i += 1
command += ['-DXNU_BOOT_PROGRESS=1', '-mgeneral-regs-only', '-Wall', '-Wextra', '-Werror',
            '-I', str(ROOT / '.cache/xnu-work/osfmk'),
            '-I', str(font_dir)]
subprocess.run(command, cwd=record['directory'], check=True, timeout=90)
(OUT / 'compile-command.json').write_text(json.dumps(command,indent=2)+'\n')
fault_source = ROOT / 'kits/sheng/BootFaultProbe.c'
fault_obj = OUT / 'BootFaultProbe.o'
fault_command = [str(fault_source) if arg == str(source) else str(fault_obj) if arg == str(obj) else arg for arg in command]
subprocess.run(fault_command, cwd=record['directory'], check=True, timeout=90)
policy_source = ROOT / 'kits/sheng/BootWatchdogPolicy.c'
policy_obj = OUT / 'BootWatchdogPolicy.o'
policy_record = json.loads((ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/bsd/DEVELOPMENT/kern_exec.o.json').read_text().strip().rstrip(','))
policy_command = []
policy_args = policy_record['arguments']
j = 0
while j < len(policy_args):
    arg = policy_args[j]
    if arg == policy_record['file']:
        policy_command.append(str(policy_source))
    elif arg in ('-o', '-MF', '-MT', '-MJ'):
        if arg == '-o': policy_command += ['-o', str(policy_obj)]
        j += 1
    elif arg == '-Xclang' and policy_args[j+1].startswith('-W'):
        j += 1
    elif not arg.startswith('-W'):
        policy_command.append(arg)
    j += 1
policy_command += ['-mgeneral-regs-only', '-Wall', '-Wextra', '-Werror']
# On the source-storage profiles (iPadOS 26/27) four of the sources below already belong to another archive that is
# force-loaded beside this one, so a second copy is a duplicate symbol: BootWatchdogPolicy.c, BootDiagExport.c and
# BootLatencyTrace.c are members of libShengSourceStorage.a (build-sheng-source-storage.py), and AdspFirmware.c is built
# by build-gop-legacy-display-27.py into libNativeLegacyDisplay.late.a. The iOS 17 profile has neither archive, so it
# needs all of them from here. Build only what this profile does not get elsewhere.
shared = (os.environ.get('XNU_SOURCE_STORAGE') == '1')
optional = []
if not shared:
    subprocess.run(policy_command, cwd=policy_record['directory'], check=True, timeout=90)
    optional.append(policy_obj)
    # The diagnostic file export (BootLog.c's sheng-diag-export) reads files through the VFS: the same BSD-side flags.
    diag_source = ROOT / 'kits/sheng/BootDiagExport.c'
    diag_obj = OUT / 'BootDiagExport.o'
    diag_command = [str(diag_source) if arg == str(policy_source) else str(diag_obj) if arg == str(policy_obj) else arg for arg in policy_command]
    subprocess.run(diag_command, cwd=policy_record['directory'], check=True, timeout=90)
    optional.append(diag_obj)
    latency_source = ROOT / 'kits/sheng/BootLatencyTrace.c'
    latency_obj = OUT / 'BootLatencyTrace.o'
    latency_command = [str(latency_source) if arg == str(source) else str(latency_obj) if arg == str(obj) else arg for arg in command]
    subprocess.run(latency_command, cwd=record['directory'], check=True, timeout=90)
    optional.append(latency_obj)
    # The firmware file reader of the audio DSP bring-up (AdspProbe.cpp) uses the VFS too.
    fw_source = ROOT / 'kits/sheng/AdspFirmware.c'
    fw_obj = OUT / 'AdspFirmware.o'
    fw_command = [str(fw_source) if arg == str(policy_source) else str(fw_obj) if arg == str(policy_obj) else arg for arg in policy_command]
    subprocess.run(fw_command, cwd=policy_record['directory'], check=True, timeout=90)
    optional.append(fw_obj)

archive = OUT / 'libBootProgress.a'
archive.unlink(missing_ok=True)
subprocess.run(['xcrun', 'ar', 'rcs', str(archive), str(obj), str(fault_obj), *map(str, optional)], check=True, timeout=30)
(OUT / 'build-report.json').write_text(json.dumps({
    'fault_sha256': hashlib.sha256(fault_source.read_bytes()).hexdigest(),
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'lifecycle_sha256': hashlib.sha256((source.parent / 'BootProgressLifecycle.h').read_bytes()).hexdigest(),
    'core_sha256': hashlib.sha256((source.parent / 'BootProgressCore.h').read_bytes()).hexdigest(),
    'display_sha256': hashlib.sha256((source.parent / 'BootProgressDisplay.h').read_bytes()).hexdigest(),
    'pulse_sha256': hashlib.sha256((source.parent / 'BootProgressPulse.h').read_bytes()).hexdigest(),
    'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
    'hardware_execution': False,
    # Absent when the profile takes these objects from another force-loaded archive (see `shared` above).
    **({'watchdog_policy_sha256': hashlib.sha256(policy_source.read_bytes()).hexdigest(),
        'latency_sha256': hashlib.sha256(latency_source.read_bytes()).hexdigest(),
        'latency_unwind_sha256': hashlib.sha256((latency_source.parent / 'BootLatencyUnwind.h').read_bytes()).hexdigest(),
        'latency_files_sha256': hashlib.sha256((latency_source.parent / 'BootLatencyFiles.h').read_bytes()).hexdigest(),
        'latency_store_sha256': hashlib.sha256((latency_source.parent / 'BootLatencyStore.h').read_bytes()).hexdigest(),
        'diag_sha256': hashlib.sha256(diag_source.read_bytes()).hexdigest()} if not shared else {}),
}, indent=2) + '\n')
print('PASS: strict source-kernel ARM64 boot diagnostic build')
