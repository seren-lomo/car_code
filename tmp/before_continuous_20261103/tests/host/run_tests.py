"""Run each mocked-HAL case against the actual chassis source in a fresh process.
Build command is recorded in README_chassis_debug.md. No hardware is accessed.
"""
import ctypes
from pathlib import Path
import subprocess
import sys

library = Path(__file__).with_name("chassis_test.dll").resolve()
if len(sys.argv) > 1:
    dll = ctypes.CDLL(str(library))
    dll.run_case.argtypes = [ctypes.c_int]
    dll.run_case.restype = ctypes.c_int
    result = dll.run_case(int(sys.argv[1]))
    print(f"case {sys.argv[1]}: {'PASS' if result == 0 else f'FAIL {result}'}")
    sys.exit(result != 0)

for case in range(1, 20):
    subprocess.run([sys.executable, __file__, str(case)], check=True)
print("19/19 position behavior cases passed (mock HAL, no hardware).")
