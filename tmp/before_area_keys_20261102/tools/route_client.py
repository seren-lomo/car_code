"""UART4 route/vision protocol. Python 3.9+, pyserial only needed for hardware.

All traffic to the STM32 must share ONE Client/serial reader and writer.
ROS: convert metres -> mm, radians -> degrees before creating a route.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import struct
import time

UPLOAD, START, STOP, WORK_DONE, STATUS, VISION = 0x10, 0x11, 0x12, 0x13, 0x14, 0x30
ACK, EVENT = 0x80, 0x81
STATES = ['EMPTY', 'READY', 'RUNNING', 'WAIT_WORK', 'DONE', 'STOPPED', 'FAULT']
EVENTS = {1:'READY', 2:'STARTED', 3:'ARRIVED', 4:'WORK_REQUIRED', 5:'DONE',
          6:'STOPPED', 7:'FAULT', 8:'POINT_DONE'}
RESULTS = ['OK', 'BAD_TYPE', 'BAD_LENGTH', 'BAD_CRC', 'BAD_VALUE', 'BUSY',
           'BAD_ID', 'BAD_STATE', 'CONFLICT']
MAX_PAYLOAD = 1300


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def frame(kind, sequence, payload=b''):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too large')
    body = struct.pack('<BBHH', 1, kind, sequence, len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class Decoder:
    """Incremental decoder: tolerates partial frames, noise and corrupt frames."""
    def __init__(self):
        self.buffer = bytearray()
        self.last_feed = time.monotonic()

    def feed(self, data):
        now = time.monotonic()
        if now - self.last_feed > 0.5:
            self.buffer.clear()
        self.last_feed = now
        self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b'\xa5\x5a':
                del self.buffer[0]
                continue
            if len(self.buffer) < 8:
                break
            version, kind, sequence, size = struct.unpack_from('<BBHH', self.buffer, 2)
            if version != 1 or size > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            end = 10 + size
            if len(self.buffer) < end:
                break
            if crc16(self.buffer[2:end-2]) != struct.unpack_from('<H', self.buffer, end-2)[0]:
                del self.buffer[0]
                continue
            messages.append((kind, sequence, bytes(self.buffer[8:end-2])))
            del self.buffer[:end]
        return messages


def integer(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}]')
    return value


def scaled(value, scale, limit, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if not -limit <= value <= limit:
        raise ValueError(f'{name} outside +/-{limit}')
    return round(value * scale)


def upload_payload(route):
    route_id = integer(route['route_id'], 1, 0xffffffff, 'route_id')
    initial = route['initial_pose']
    points = route['points']
    integer(len(points), 1, 64, 'point count')
    def pose(point):
        return (scaled(point['x_mm'], 1, 100000, 'x_mm'),
                scaled(point['y_mm'], 1, 100000, 'y_mm'),
                scaled(point['yaw_deg'], 100, 180, 'yaw_deg'))
    payload = struct.pack('<IHHiii', route_id, len(points), 0, *pose(initial))
    for point in points:
        move = integer(point.get('move_rpm', 40), 1, 120, 'move_rpm')
        turn = integer(point.get('turn_rpm', 20), 1, 120, 'turn_rpm')
        work = integer(point.get('work_id', 0), 0, 65535, 'work_id')
        payload += struct.pack('<iiiHHHH', *pose(point), move, turn, work, int(work != 0))
    return payload


def status_payload(payload):
    if len(payload) != 36:
        raise ValueError('expected 36-byte status')
    fields = struct.unpack('<BBBBIHHHBBIIiihH', payload)
    names = ('request', 'code', 'state', 'phase', 'route_id', 'point_index',
             'point_count', 'work_id', 'chassis_fault', 'fault_address', 'tick',
             'fault', 'x_mm', 'y_mm', 'yaw_cdeg', 'reserved')
    result = dict(zip(names, fields))
    result['state_name'] = STATES[result['state']] if result['state'] < len(STATES) else 'UNKNOWN'
    result['yaw_deg'] = result['yaw_cdeg'] / 100
    return result


class Client:
    """Call from one thread/process. ACK retries reuse the exact sequence/frame.

    The UART is owned by this object; vision uses this same instance too.
    Events can precede an ACK. Always poll status if an event was missed.
    """
    def __init__(self, serial_port):
        self.serial = serial_port
        self.decoder = Decoder()
        self.sequence = 0
        self.events = deque(maxlen=256)
        self.pending = deque()

    def receive(self, deadline):
        while time.monotonic() < deadline:
            if self.pending:
                return self.pending.popleft()
            data = self.serial.read(max(1, min(self.serial.in_waiting, 4096)))
            if data:
                self.pending.extend(self.decoder.feed(data))
        return None

    def request(self, kind, payload=b'', retries=3, timeout=1.0):
        self.sequence = (self.sequence + 1) & 0xffff
        packet = frame(kind, self.sequence, payload)
        for _ in range(retries):
            if self.serial.write(packet) != len(packet):
                raise IOError('short UART write')
            deadline = time.monotonic() + timeout
            while True:
                reply = self.receive(deadline)
                if reply is None:
                    break
                typ, seq, body = reply
                if typ not in (ACK, EVENT):
                    continue
                status = status_payload(body)
                if typ == EVENT:
                    self.events.append(status)
                elif seq == self.sequence and status['request'] == kind:
                    if status['code']:
                        code = status['code']
                        label = RESULTS[code] if code < len(RESULTS) else str(code)
                        raise RuntimeError(f'STM32 rejected request: {label}; {status}')
                    return status
        raise TimeoutError('No matching ACK; query STATUS before taking further action')

    def upload(self, route):
        return self.request(UPLOAD, upload_payload(route))

    def start(self, route_id):
        return self.request(START, struct.pack('<I', route_id))

    def stop(self, route_id=0):
        return self.request(STOP, struct.pack('<I', route_id))

    def status(self):
        return self.request(STATUS)

    def work_done(self, route_id, point_index, success=True):
        return self.request(WORK_DONE, struct.pack('<IHBB', route_id, point_index, int(success), 0))

    def vision(self, route_id, point_index, work_id, dx_mm, dy_mm, dyaw_deg, confidence):
        """Body offsets: +dx front, +dy left, +dyaw CCW; no automatic motion."""
        payload = struct.pack('<IHHii hH', route_id, point_index, work_id,
                              scaled(dx_mm, 1, 100000, 'dx_mm'),
                              scaled(dy_mm, 1, 100000, 'dy_mm'),
                              scaled(dyaw_deg, 100, 180, 'dyaw_deg'),
                              integer(confidence, 0, 1000, 'confidence'))
        return self.request(VISION, payload)

    def run(self, route, on_work=None):
        """on_work(client, status) -> True only after the work REALLY finishes.

        If omitted, wait indefinitely at work points (no fake completion).
        A callback may send vision on this same Client. It must monitor its
        own manipulator errors/stop requests before returning completion.
        """
        self.upload(route)
        self.start(route['route_id'])
        last_state = None
        handled = set()
        while True:
            status = self.status()  # recovers if WORK_REQUIRED event was dropped
            if status['route_id'] != route['route_id']:
                raise RuntimeError('Active route was replaced or MCU rebooted')
            key = (status['state'], status['point_index'], status['fault'])
            if key != last_state:
                print(json.dumps(status, ensure_ascii=False), flush=True)
                last_state = key
            if status['state'] in (4, 5, 6):
                return status
            if status['state'] == 3 and key not in handled and on_work is not None:
                handled.add(key)
                success = on_work(self, status)
                self.work_done(status['route_id'], status['point_index'], success is True)
            time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='/dev/serial0 or COMx; 115200 8N1')
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('upload', 'run'):
        command = sub.add_parser(name)
        command.add_argument('json_file', type=Path)
        if name == 'run':
            command.add_argument('--interactive', action='store_true', help='manually confirm real work completion')
    sub.add_parser('status')
    sub.add_parser('stop')
    command = sub.add_parser('start')
    command.add_argument('route_id', type=int)
    command = sub.add_parser('work-done')
    command.add_argument('route_id', type=int)
    command.add_argument('point_index', type=int)
    command.add_argument('--failed', action='store_true')
    args = parser.parse_args()
    import serial
    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1) as port:
        client = Client(port)
        try:
            if args.command in ('run', 'upload'):
                route = json.loads(args.json_file.read_text(encoding='utf-8'))
                if args.command == 'upload':
                    print(client.upload(route))
                else:
                    def manual(_client, status):
                        return input(f"point {status['point_index']}, work {status['work_id']}: enter done after completion, or fail: ").strip() == 'done'
                    print(client.run(route, manual if args.interactive else None))
            elif args.command == 'start':
                print(client.start(args.route_id))
            elif args.command == 'work-done':
                print(client.work_done(args.route_id, args.point_index, not args.failed))
            else:
                print(getattr(client, args.command)())
        except KeyboardInterrupt:
            print(client.stop())


if __name__ == '__main__':
    main()
