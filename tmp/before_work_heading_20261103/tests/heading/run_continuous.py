"""Real route parser/executor + real path/heading controller, simulated wheels/IMU.
Each case runs in a fresh process so static firmware state starts at power-on.
"""
import ctypes as C
import json
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
import route_client as P


def run(name):
    lib = C.CDLL(str(Path(__file__).with_name('heading_test.dll')))
    lib.plant_get.argtypes = [C.c_uint32]
    lib.plant_get.restype = C.c_uint32
    lib.plant_float.argtypes = [C.c_uint32]
    lib.plant_float.restype = C.c_float
    lib.plant_step.argtypes = [C.c_uint32]
    lib.plant_control.argtypes = [C.c_uint32, C.c_float]
    lib.plant_feed.argtypes = [C.c_void_p, C.c_uint16]
    lib.plant_init()
    seq = 0

    def send(kind, payload=b''):
        nonlocal seq
        seq += 1
        packet = P.frame(kind, seq, payload)
        lib.plant_feed(packet, len(packet))
        lib.plant_step(5)

    if name == 'optimized16':
        lib.plant_select_area()
        for packet in json.loads((ROOT/'tests/heading/optimized16_frames.json').read_text()):
            raw = bytes.fromhex(packet)
            lib.plant_feed(raw, len(raw))
            lib.plant_step(5)
        route_id = 1793701103
        expected = [1, 3, 4, 6, 8, 9, 11, 15]
        point_count = 16
    else:
        points = {
            'straight': [(0, 200, 100, 0), (0, 930, 100, 0), (0, 1050, 20, 1)],
            'corner': [(500, 0, 100, 0), (500, 500, 100, 0), (1000, 500, 20, 1)],
            'reverse': [(500, 0, 40, 0), (0, 0, 30, 1)],
            'duplicate': [(0, 0, 40, 0), (200, 0, 40, 0), (200, 0, 40, 0), (500, 0, 20, 1)],
            'shortfast': [(0, 10, 120, 0), (0, 20, 120, 0), (0, 300, 120, 0),
                          (0, 310, 120, 0), (0, 320, 120, 0), (0, 600, 20, 1)],
            'stop': [(0, 1000, 100, 0), (500, 1000, 20, 1)],
            'imu_lost': [(0, 1000, 100, 0), (500, 1000, 20, 1)],
            'feedback_lost': [(0, 1000, 100, 0), (500, 1000, 20, 1)],
            'yaw_correction': [(0, 500, 60, 0), (500, 500, 60, 1)],
            'calibration': [(0, 500, 60, 0), (500, 500, 60, 1)],
            'heading_change': [(200, 0, 40, 0), (200, 200, 40, 1)],
            'work_boundary': [(200, 0, 40, 1), (200, 0, 40, 2), (500, 0, 40, 3)],
            'pass_end': [(200, 0, 40, 0), (500, 0, 40, 0)],
            'shortcorner': [(200, 0, 120, 0), (210, 0, 120, 0),
                            (220, 0, 120, 0), (220, 300, 40, 1)],
        }[name]
        if name == 'calibration':
            lib.plant_control(5, 1.1)
            lib.plant_control(6, 0.9)
        r = dict(route_id=1, initial_pose=dict(x_mm=0, y_mm=0, yaw_deg=0), points=[
            dict(x_mm=x, y_mm=y, yaw_deg=(90 if name == 'heading_change' and i else 0),
                 move_rpm=rpm, turn_rpm=15, work_id=work)
            for i, (x, y, rpm, work) in enumerate(points)])
        send(P.UPLOAD, P.upload_payload(r))
        send(P.START, struct.pack('<I', 1))
        route_id = 1
        expected = [i for i, point in enumerate(points) if point[3]]
        point_count = len(points)

    assert lib.plant_get(0) == 2, (name, 'route did not start', lib.plant_get(0))
    if name in ('stop', 'imu_lost', 'feedback_lost'):
        lib.plant_step(1500)
        assert lib.plant_get(0) == 2 and lib.plant_float(2) > 5
        lib.plant_control({'stop': 1, 'imu_lost': 2, 'feedback_lost': 3}[name], 0)
        lib.plant_step(1000)
        assert lib.plant_get(0) == (5 if name == 'stop' else 6)
        assert not lib.plant_get(11) and not lib.plant_get(12), 'queued motion survived stop'
        syncs = lib.plant_get(14)
        lib.plant_step(3000)
        assert lib.plant_get(14) == syncs and lib.plant_get(6) == 0, 'unexpected restart'
        print(f'{name}: PASS (stopped, no automatic restart)')
        return
    if name == 'yaw_correction':
        lib.plant_step(800)
        lib.plant_control(7, 0)
        lib.plant_control(4, 4)

    stations, transitions = [], []
    previous_index = lib.plant_get(1)
    batch_preparation_stops = 8
    max_corridor = 0.0
    for _ in range(80000):
        lib.plant_step(10)
        state, index = lib.plant_get(0), lib.plant_get(1)
        if state == 6:
            raise AssertionError(f'{name}: FAULT route={lib.plant_get(2)} chassis={lib.plant_get(3)} '
                                 f'point={index} node={lib.plant_get(8)} remaining={lib.plant_float(7)}')
        if name == 'corner':
            x, y = lib.plant_float(0), lib.plant_float(1)

            def line_distance(ax, ay, bx, by):
                t = max(0, min(1, ((x-ax)*(bx-ax)+(y-ay)*(by-ay))/((bx-ax)**2+(by-ay)**2)))
                return ((x-ax-t*(bx-ax))**2+(y-ay-t*(by-ay))**2)**0.5

            max_corridor = max(max_corridor, min(line_distance(0, 0, 500, 0),
                               line_distance(500, 0, 500, 500), line_distance(500, 500, 1000, 500)))
        if index != previous_index:
            if previous_index not in expected and state == 2:
                transitions.append((previous_index, lib.plant_float(2)))
                # Never re-prepare or send FE at a pass point in a fixed-heading batch.
                if name != 'heading_change':
                    assert lib.plant_get(4) == batch_preparation_stops, (name, 'stop at pass point')
                    if name != 'reverse':
                        assert lib.plant_float(2) > 0.5, (name, 'zero speed at pass point')
            previous_index = index
        if state == 3:
            stations.append(index)
            assert abs(lib.plant_float(9)) <= 1.1 and abs(lib.plant_float(10)) <= 1.1
            lib.plant_step(2000)
            assert lib.plant_get(0) == 3, 'work gate released without completion'
            send(P.WORK_DONE, struct.pack('<IHBB', route_id, index, 1, 0))
            # The next batch sends four preparation stops after the work gate.
            lib.plant_step(100)
            batch_preparation_stops = lib.plant_get(4)
            previous_index = lib.plant_get(1)
        if lib.plant_get(0) == 4:
            break
    assert lib.plant_get(0) == 4, (name, 'timeout', lib.plant_get(1), lib.plant_float(7))
    assert stations == expected, (name, stations)
    assert lib.plant_get(7) == point_count and lib.plant_get(6) == len(expected)
    if name != 'heading_change':
        batches = 2 if name == 'work_boundary' else (1 if name == 'pass_end' else len(expected))
        assert lib.plant_get(5) == batches, (name, 'extra stop/completed action', lib.plant_get(5))
    assert lib.plant_get(13) <= 120 and not lib.plant_get(16)
    if name == 'optimized16':
        assert abs(lib.plant_float(3)-150) <= 6 and abs(lib.plant_float(4)-150) <= 6
    if name == 'straight':
        assert transitions[0][1] > 80 and transitions[1][1] > 5, transitions
    if name == 'calibration':
        assert abs(lib.plant_float(3)-500) <= 6 and abs(lib.plant_float(4)-500) <= 6
        assert abs(lib.plant_float(0)-550) <= 8 and abs(lib.plant_float(1)-450) <= 8
    if name == 'yaw_correction':
        assert abs(lib.plant_float(8)) <= 1.01
    if name == 'corner':
        assert max_corridor < 10, max_corridor
        print(f'corner: simulated maximum polyline deviation {max_corridor:.2f}mm')
    print(f'{name}: PASS, waits={len(stations)}, completed actions={lib.plant_get(5)}, '
          f'simulated time={lib.plant_get(10)/1000:.2f}s, pass speeds={transitions}')


if __name__ == '__main__':
    if len(sys.argv) > 1:
        run(sys.argv[1])
    else:
        for case in ('straight', 'corner', 'reverse', 'duplicate', 'shortfast', 'optimized16',
                     'stop', 'imu_lost', 'feedback_lost', 'yaw_correction', 'calibration', 'heading_change',
                     'work_boundary', 'pass_end', 'shortcorner'):
            subprocess.run([sys.executable, __file__, case], check=True)
