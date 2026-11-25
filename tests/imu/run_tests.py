"""Exercise the production parser and UART callbacks against a HAL mock."""
import ctypes as C
from pathlib import Path
import struct

lib=C.CDLL(str(Path(__file__).with_name('imu_test.dll').resolve()))
class Data(C.Structure):
    _fields_=[('angle',C.c_float*3),('gyro',C.c_float*3),('acc',C.c_float*3),
              ('yaw',C.c_float),('total',C.c_float),('count',C.c_uint32),
              ('tick',C.c_uint32),('gyro_tick',C.c_uint32),('acc_tick',C.c_uint32),
              ('mask',C.c_uint8),('online',C.c_uint8)]
class Debug(C.Structure):
    _fields_=[(name,C.c_uint32) for name in
              ('bytes','frames','checksums','overflows','uart_errors','last_error',
               'rearm_errors','continuity_resets','init_status')]
debug=Debug.in_dll(lib,'imu_debug')
watch=Data.in_dll(lib,'imu_data')
lib.IMU_GetData.argtypes=[C.POINTER(Data)]
lib.IMU_GetData.restype=C.c_uint8
lib.test_feed.argtypes=[C.c_char_p,C.c_uint32]
lib.test_tick.argtypes=[C.c_uint32]

def packet(kind=0x53, values=(0,0,0)):
    raw=bytes([0x55,kind])+struct.pack('<hhhh',*values,0)
    return raw+bytes([sum(raw)&255])
def feed(p): lib.test_feed(p,len(p))
def data():
    d=Data()
    good=lib.IMU_GetData(C.byref(d))
    assert good==d.online
    return d
def near(a,b): assert abs(a-b)<0.02,(a,b)
def reset(): lib.test_reset()
def process(p): feed(p); lib.IMU_Task(); return data()

reset()
assert not data().online and debug.init_status==0
# Fragmentation, noise, signed values, scaling, independent optional outputs.
p=packet(values=(16384,-8192,8192))
feed(b'noise'+p[:4]); lib.IMU_Task(); assert data().count==0
d=process(p[4:]); assert d.online and d.count==1
near(d.angle[0],90); near(d.angle[1],-45); near(d.yaw,45)
assert d.mask==4 and debug.bytes==16
d=process(packet(0x52,(16384,-16384,8192))+packet(0x51,(2048,-2048,0)))
near(d.gyro[0],1000); near(d.gyro[1],-1000); near(d.acc[0],1)
assert d.mask==7 and d.count==1
# Checksum corruption and a truncated frame with the following header retained.
reset(); bad=bytearray(packet()); bad[-1]^=1
d=process(bytes(bad)+packet(values=(0,0,16384)))
assert debug.checksums>=1 and d.count==1; near(d.yaw,90)
reset(); d=process(packet()[:5]+packet(values=(0,0,-16384)))
assert d.count==1; near(d.yaw,-90)
# Wrap crossing in both directions; no 360-degree jump.
reset(); process(packet(values=(0,0,32586)))
d=process(packet(values=(0,0,-32586))); near(d.total,181)
d=process(packet(values=(0,0,32586))); near(d.total,179)
lib.IMU_ZeroYaw(); d=process(packet(values=(0,0,32586)))
near(d.yaw,0); near(d.total,0)
d=process(packet(values=(0,0,-32586))); near(d.yaw,2); near(d.total,2)
# Freshness depends on angles, not gyro/unknown frames; getter checks time itself.
lib.test_tick(201); assert not data().online
d=process(packet(0x52)); assert not d.online
lib.test_tick(202); d=process(packet(values=(0,0,0)))
assert d.online and debug.continuity_resets==1
# Backlogged bytes keep their ISR timestamp; unsigned tick wrap is supported.
reset(); feed(packet()); lib.test_tick(500); lib.IMU_Task(); assert not data().online
reset(); lib.test_tick(0xfffffff0); process(packet())
lib.test_tick(0x20); assert data().online
lib.test_tick(0x100); assert not data().online
# Queue overflow invalidates old angle and resynchronizes on a subsequent frame.
reset(); process(packet()); feed(bytes(600)); lib.IMU_Task()
assert debug.overflows>0 and not data().online
assert process(packet()).online
# HAL overrun ends receive; non-blocking noise leaves receive armed.
for end_rx in (0,1):
    reset(); process(packet()); lib.test_error(8,end_rx)
    assert not data().online
    lib.IMU_Task(); assert not data().online
    assert process(packet()).online and debug.uart_errors==1
# A failed rearm is retried by the consumer task without wedging UART5.
reset(); lib.test_fail_rearm(); feed(b'\x55'); lib.IMU_Task()
assert debug.rearm_errors==1 and process(packet()).online
before=(debug.bytes,debug.uart_errors); lib.test_other_uart()
assert before==(debug.bytes,debug.uart_errors)
print('PASS: scaling, fragmentation, checksum/resync, yaw wrap/zero, freshness, '
      'tick wrap, overflow, UART errors, rearm retry, UART isolation')
