"""Actual controller + actual IMU parser with asynchronous CAN/plant mock."""
import ctypes as C
from pathlib import Path
import subprocess
import sys

if len(sys.argv)>1:
    lib=C.CDLL(str(Path(__file__).with_name('heading_test.dll').resolve()))
    lib.run_case.argtypes=[C.c_int]
    lib.run_case.restype=C.c_int
    case=int(sys.argv[1])
    result=lib.run_case(case)
    if result:
        print(f'case {case}: FAIL {result}')
    else:
        distance=C.c_float.in_dll(lib,'result_distance').value
        yaw=C.c_float.in_dll(lib,'result_yaw').value
        elapsed=C.c_uint32.in_dll(lib,'result_ms').value
        print(f'case {case}: PASS (wheel distance={distance:.2f} mm, '
              f'yaw error={yaw:.3f} deg, t={elapsed} ms)')
    sys.exit(result!=0)

for case in range(1,29):
    subprocess.run([sys.executable,__file__,str(case)],check=True)
print('28/28 heading + distance behavior cases passed (simulation, no hardware).')
