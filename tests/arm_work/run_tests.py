"""Run actual ARM-compiled CAN driver against a motor/CAN mock in a CPU emulator.

Local test dependencies: python -m pip install --target tmp/arm_test_runtime unicorn pyelftools
"""
from pathlib import Path
import subprocess
import sys
import argparse

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=(0, 1), type=int, default=0)
parser.add_argument('--tray', action='store_true', help='test transfer with the configured tray coordinates')
args = parser.parse_args()
mode = args.mode
tray_define = f'-DARM_TRAY_ENABLED={int(args.tray)}'
sys.path.insert(0, str(ROOT / 'tmp/arm_test_runtime'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3

CLANG = Path('E:/APP/Matlab25/toolbox/sldrt/clang/win64/clang.exe')
LLD = CLANG.with_name('lld.exe')
BUILD = ROOT / 'tmp/arm_work_tests' / f'mode{mode}_tray{int(args.tray)}'
BUILD.mkdir(parents=True, exist_ok=True)
objects = []
for source in ('Core/Src/arm_can.c', 'Core/Src/arm_work.c', 'Core/Src/route_keys.c', 'tests/arm_work/harness.c'):
    obj = BUILD / (Path(source).stem + '.o')
    subprocess.run([str(CLANG), '--target=arm-none-eabi', '-mcpu=cortex-m3', '-mthumb',
                    '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror', '-ffreestanding',
                    '-fno-builtin', f'-DARM_VISION_TEST_MODE={mode}', tray_define,
                    '-include', 'tests/arm_can/mock.h', '-ICore/Inc',
                    '-c', source, '-o', str(obj)], cwd=ROOT, check=True)
    objects.append(str(obj))
elf_path = BUILD / 'tests.elf'
subprocess.run([str(LLD), '-flavor', 'gnu', '-m', 'armelf', '-Ttext=0x10000',
                '--entry=run_case', '-o', str(elf_path), *objects], cwd=ROOT, check=True)
with elf_path.open('rb') as stream:
    elf = ELFFile(stream)
    segments = [(p['p_vaddr'], p.data()) for p in elf.iter_segments() if p['p_type'] == 'PT_LOAD']
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

def run(case):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0, 0x200000)
    for address, data in segments:
        cpu.mem_write(address, data)
    finish = 0x1f0000
    cpu.reg_write(UC_ARM_REG_SP, 0x1e0000)
    cpu.reg_write(UC_ARM_REG_LR, finish | 1)
    cpu.reg_write(UC_ARM_REG_R0, case)
    helpers = {symbols[n] & ~1: n for n in ('__aeabi_ldivmod', '__aeabi_uldivmod') if n in symbols}
    def hook(uc, address, size, user):
        if address == finish:
            uc.emu_stop()
        elif address in helpers:
            a = uc.reg_read(UC_ARM_REG_R0) | uc.reg_read(UC_ARM_REG_R1) << 32
            b = uc.reg_read(UC_ARM_REG_R2) | uc.reg_read(UC_ARM_REG_R3) << 32
            if helpers[address] == '__aeabi_ldivmod':
                if a >= 1 << 63: a -= 1 << 64
                if b >= 1 << 63: b -= 1 << 64
            quotient = abs(a) // abs(b)
            if (a < 0) != (b < 0): quotient = -quotient
            remainder = a - quotient * b
            for reg, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3),
                                  (quotient, quotient >> 32, remainder, remainder >> 32)):
                uc.reg_write(reg, value & 0xffffffff)
            uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
    # Only division helpers and the return sentinel need Python callbacks.
    for address in (*helpers, finish):
        cpu.hook_add(UC_HOOK_CODE, hook, begin=address, end=address)
    cpu.emu_start(symbols['run_case'] | 1, finish + 2, count=150000000)
    if cpu.reg_read(UC_ARM_REG_PC) != finish:
        raise AssertionError(f'case {case}: emulator did not finish')
    result = cpu.reg_read(UC_ARM_REG_R0)
    assert result == 0, f'case {case}: failed assertion {result}'
    print(f'case {case}: PASS')

if args.tray:
    cases = list(range(94, 113))
else:
    cases = list(range(1, 43) if mode == 0 else range(50, 60)) + list(range(70, 100)) + list(range(103, 106)) + list(range(108, 113))
for case in cases:
    run(case)
print(f'{len(cases)}/{len(cases)} arm workflow cases passed (mode={mode}, tray={args.tray}); hardware not tested.')
