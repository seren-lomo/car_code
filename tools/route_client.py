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
AREA_ACK, UPLOAD_FOR_AREA, START_AREA = 0x15, 0x16, 0x82
ACK, EVENT = 0x80, 0x81
ARM_COMMAND, ARM_STATUS, ARM_EVENT = 0x31, 0x32, 0x83
ARM_STATES = ['NEED_ORIGIN', 'ZEROING', 'READY', 'PREPARING', 'WAIT_VISION',
              'ALIGNING', 'ALIGNED', 'LOWERING', 'WAIT_GRIP', 'LIFTING', 'PICKED',
              'PARKING', 'READY_LEAVE', 'STOPPED', 'FAULT', 'DEBUG_MOVING', 'DEBUG_HOLD',
              'TRAY_LOADING', 'TRAY_UNLOADING', 'PLACED']
STATES = ['EMPTY', 'READY', 'RUNNING', 'WAIT_WORK', 'DONE', 'STOPPED', 'FAULT']
EVENTS = {1:'READY', 2:'STARTED', 3:'ARRIVED', 4:'WORK_REQUIRED', 5:'DONE',
          6:'STOPPED', 7:'FAULT', 8:'POINT_DONE'}
RESULTS = ['OK', 'BAD_TYPE', 'BAD_LENGTH', 'BAD_CRC', 'BAD_VALUE', 'BUSY',
           'BAD_ID', 'BAD_STATE', 'CONFLICT']
MAX_PAYLOAD = 1304


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


def start_area_payload(payload):
    if len(payload) != 12:
        raise ValueError('expected 12-byte start area signal')
    request_id, tick, area, key, reserved = struct.unpack('<IIBBH', payload)
    if not request_id or area not in (1, 2) or key != (0 if area == 2 else 1) or reserved:
        raise ValueError('invalid start area signal')
    return dict(request_id=request_id, tick=tick, area=area, key=key)


def arm_command_payload(route_id, point_index, work_id, command_id, opcode,
                        z_deg=0, yaw_deg=0, reach_deg=0, reference_id=0, destination_slot=0):
    """Absolute motor-shaft angles from the recorded manual pose."""
    slot = integer(destination_slot, 1, 3, 'destination_slot') if opcode in (7, 8) else integer(
        destination_slot, 0, 0, 'destination_slot')
    return struct.pack('<IHHIBBHiiiI',
                       integer(route_id, 0, 0xffffffff, 'route_id'),
                       integer(point_index, 0, 65535, 'point_index'),
                       integer(work_id, 0, 65535, 'work_id'),
                       integer(command_id, 1, 0xffffffff, 'command_id'),
                       integer(opcode, 1, 8, 'opcode'), slot, 0,
                       scaled(z_deg, 100, 21474836, 'z_deg'),
                       scaled(yaw_deg, 100, 21474836, 'yaw_deg'),
                       scaled(reach_deg, 100, 21474836, 'reach_deg'),
                       integer(reference_id, 0, 0xffffffff, 'reference_id'))


def arm_status_payload(payload):
    if len(payload) != 52:
        raise ValueError('expected 52-byte arm status')
    fields = struct.unpack('<IHHIIBBBBHHiiiiiiI', payload)
    names = ('route_id', 'point_index', 'work_id', 'command_id', 'aligned_command_id',
             'state', 'fault', 'active_axis', 'opcode', 'flags', 'reserved',
             'z_cdeg', 'yaw_cdeg', 'reach_cdeg',
             'target_z_cdeg', 'target_yaw_cdeg', 'target_reach_cdeg', 'tick')
    result = dict(zip(names, fields))
    result['state_name'] = ARM_STATES[result['state']] if result['state'] < len(ARM_STATES) else 'UNKNOWN'
    result['test_mode'] = bool(result['flags'] & 8)
    result['tray_enabled'] = bool(result['flags'] & 16)
    result['tray_uncertain'] = bool(result['flags'] & 32)
    result['motor_fault'] = (result['flags'] >> 6) & 7
    result['motor_fault_stage'] = (result['flags'] >> 9) & 15
    result['motor_fault_flags'] = ((result['flags'] >> 13) & 1) | ((result['flags'] >> 12) & 0x0c)
    result['tray_count'] = result['reserved'] & 0xff
    result['destination_slot'] = result['reserved'] >> 8
    for name in ('z', 'yaw', 'reach', 'target_z', 'target_yaw', 'target_reach'):
        result[name + '_deg'] = result[name + '_cdeg'] / 100
    return result


class Rejected(RuntimeError):
    def __init__(self, status):
        self.status = status
        self.code = status['code']
        label = RESULTS[self.code] if self.code < len(RESULTS) else str(self.code)
        super().__init__(f'STM32 rejected request: {label}; {status}')


class Client:
    """Call from one thread/process. ACK retries reuse the exact sequence/frame.

    The UART is owned by this object; vision uses this same instance too.
    Events can precede an ACK. Always poll status if an event was missed.
    """
    def __init__(self, serial_port):
        self.serial = serial_port
        self.decoder = Decoder()
        self.sequence = 0
        self.arm_events = deque(maxlen=256)
        self.arm_generation = 0
        self.events = deque(maxlen=256)
        self.pending = deque()
        self.start_areas = deque(maxlen=32)
        self.handled_areas = deque(maxlen=64)

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
                if typ == ARM_EVENT:
                    self.arm_events.append(arm_status_payload(body))
                    self.arm_generation += 1
                    continue
                if typ == START_AREA:
                    self.start_areas.append(start_area_payload(body))
                    continue
                if typ not in (ACK, EVENT):
                    continue
                status = status_payload(body)
                if typ == EVENT:
                    self.events.append(status)
                elif seq == self.sequence and status['request'] == kind:
                    if status['code']:
                        raise Rejected(status)
                    return status
        raise TimeoutError('No matching ACK; query STATUS before taking further action')

    def upload(self, route, start_request_id=None):
        payload = upload_payload(route)
        if start_request_id is not None:
            integer(start_request_id, 1, 0xffffffff, 'start_request_id')
            return self.request(UPLOAD_FOR_AREA, struct.pack('<I', start_request_id) + payload)
        return self.request(UPLOAD, payload)

    def acknowledge_area(self, signal):
        return self.request(AREA_ACK, struct.pack('<IB3x', signal['request_id'], signal['area']))

    def wait_start_area(self, timeout=None):
        """Wait for KEY1=area1 / KEY0=area2. ACK and return BEFORE scanning.

        Repeat signals are acknowledged but returned only once per Client.
        Reuse this same Client for planning, upload, vision and work completion.
        """
        deadline = float('inf') if timeout is None else time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.start_areas:
                signal = self.start_areas.popleft()
                identity = (signal['request_id'], signal['tick'], signal['area'])
                if identity in self.handled_areas:
                    continue
                try:
                    self.acknowledge_area(signal)
                except Rejected as error:
                    if error.code in (6, 7):  # superseded or locally cancelled
                        continue
                    raise
                self.handled_areas.append(identity)
                return signal
            reply = self.receive(min(deadline, time.monotonic() + 0.2))
            if reply is None:
                continue
            typ, seq, body = reply
            if typ == START_AREA:
                self.start_areas.append(start_area_payload(body))
            elif typ == EVENT:
                self.events.append(status_payload(body))
            elif typ == ARM_EVENT:
                self.arm_events.append(arm_status_payload(body))
                self.arm_generation += 1
        raise TimeoutError('No new start-area key signal')

    def run_after_key(self, scan_and_plan, on_work=None):
        """scan_and_plan(signal) must return an absolute route dictionary.
        A new area selection invalidates an old planner result on the STM32.
        """
        signal = self.wait_start_area()
        print(f"KEY{signal['key']}: start area {signal['area']}, request {signal['request_id']}", flush=True)
        route = scan_and_plan(signal)  # Radar scanning/planning starts HERE.
        return self.run(route, on_work, start_request_id=signal['request_id'])

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

    def arm_command(self, route_id, point_index, work_id, command_id, opcode,
                    z_deg=0, yaw_deg=0, reach_deg=0, reference_id=0, destination_slot=0):
        """ACK means accepted, not motion complete. Wait for the ARM state too."""
        return self.request(ARM_COMMAND, arm_command_payload(
            route_id, point_index, work_id, command_id, opcode,
            z_deg, yaw_deg, reach_deg, reference_id, destination_slot))

    def arm_zero(self, command_id):
        """Record the manually positioned pose; this is NOT automatic homing."""
        return self.arm_command(0, 0, 0, command_id, 1)

    def arm_test_begin(self, test_id, work_id):
        """Stationary firmware only; IDs increase for every new test session."""
        integer(test_id, 1, 0xffffffff, 'test_id')
        integer(work_id, 102, 107, 'work_id')
        return self.arm_command(test_id, 0, work_id, 1, 6)

    def arm_align(self, route_id, point_index, work_id, command_id, z_deg, yaw_deg, reach_deg):
        return self.arm_command(route_id, point_index, work_id, command_id, 2,
                                z_deg, yaw_deg, reach_deg)

    def arm_pick(self, route_id, point_index, work_id, command_id, aligned_command_id):
        return self.arm_command(route_id, point_index, work_id, command_id, 3,
                                reference_id=aligned_command_id)

    def arm_grip_done(self, route_id, point_index, work_id, command_id, aligned_command_id):
        """Only after the external claw operation actually completed."""
        return self.arm_command(route_id, point_index, work_id, command_id, 4,
                                reference_id=aligned_command_id)

    def arm_finish(self, route_id, point_index, work_id, command_id):
        return self.arm_command(route_id, point_index, work_id, command_id, 5)

    def arm_tray_place(self, route_id, point_index, work_id, command_id, slot,
                       z_deg, yaw_deg, reach_deg):
        """Place the oldest tray item at the vision-selected slot and release pose."""
        return self.arm_command(route_id, point_index, work_id, command_id, 7,
                                z_deg, yaw_deg, reach_deg, destination_slot=slot)

    def arm_tray_reload_pick(self, route_id, point_index, work_id, command_id, slot,
                             pick_z_deg, aligned_command_id):
        """Lower onto a previously placed processing item to load it back onto the tray."""
        return self.arm_command(route_id, point_index, work_id, command_id, 8,
                                z_deg=pick_z_deg, reference_id=aligned_command_id,
                                destination_slot=slot)

    def arm_status(self):
        generation = self.arm_generation
        self.request(ARM_STATUS)
        # Firmware queues the arm report BEFORE the matching ACK, on the same UART.
        if self.arm_generation == generation:
            raise TimeoutError('Arm report missing; query ARM_STATUS again, do not resend motion with a new ID')
        return self.arm_events[-1]

    def wait_arm(self, states, context=None, command_id=None, timeout=120):
        """states: arm names; context: optional (route_id, index, work_id).

        Stops/FAULT are terminal. Timeout leaves outcome unknown: inspect status
        and stop as needed, never issue a replacement pick with a fresh ID.
        """
        deadline = time.monotonic() + timeout
        if isinstance(states, str):
            states = {states}
        while time.monotonic() < deadline:
            arm = self.arm_status()
            if arm['state_name'] in ('STOPPED', 'FAULT'):
                raise RuntimeError(f'Arm stopped: {arm}')
            if context is not None:
                if tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) != tuple(context):
                    raise RuntimeError(f'Arm context no longer active: {arm}')
                if not arm['test_mode']:
                    route = self.status()
                    if route['state'] != 3 or tuple(route[k] for k in ('route_id', 'point_index', 'work_id')) != tuple(context):
                        raise RuntimeError(f'Work context no longer active: {route}')
            if (arm['state_name'] in states and
                    (context is None or tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) == tuple(context)) and
                    (command_id is None or arm['command_id'] == command_id)):
                return arm
            time.sleep(0.05)
        raise TimeoutError('Arm action incomplete; query ARM_STATUS and STATUS before retrying')

    def run(self, route, on_work=None, start_request_id=None):
        """on_work(client, status) -> True only after the work REALLY finishes.

        If omitted, wait indefinitely at work points (no fake completion).
        A callback may send vision on this same Client. It must monitor its
        own manipulator errors/stop requests before returning completion.
        """
        self.upload(route, start_request_id)
        # A short preset plan may finish before the operator releases the key.
        # The MCU inhibits motion while either key is held. Wait for release
        # (and a fresh IMU), while respecting cancellation/reselection states.
        start_deadline = time.monotonic() + (10.0 if start_request_id is not None else 0.0)
        while True:
            try:
                self.start(route['route_id'])
                break
            except Rejected as error:
                if (error.code != 7 or error.status['state'] != 1 or
                        error.status['route_id'] != route['route_id'] or
                        time.monotonic() >= start_deadline):
                    raise
                time.sleep(0.05)
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
        command.add_argument('--start-request-id', type=int, help='token returned by wait-area')
        if name == 'run':
            command.add_argument('--interactive', action='store_true', help='manually confirm real work completion')
    sub.add_parser('status')
    sub.add_parser('arm-status')
    command = sub.add_parser('arm-zero', help='record manually set initial pose, no automatic homing')
    command.add_argument('command_id', type=int)
    command = sub.add_parser('arm-test-begin', help='stationary vision/arm test; requires test firmware')
    command.add_argument('test_id', type=int)
    command.add_argument('work_id', type=int, choices=range(102, 108))
    sub.add_parser('stop')
    sub.add_parser('wait-area', help='wait and acknowledge a key; prints area and request_id')
    command = sub.add_parser('wait-run', help='key selects one of two preset routes (communication test)')
    command.add_argument('--area1', required=True, type=Path)
    command.add_argument('--area2', required=True, type=Path)
    command.add_argument('--interactive', action='store_true')
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
            def manual(_client, status):
                return input(f"point {status['point_index']}, work {status['work_id']}: enter done after completion, or fail: ").strip() == 'done'
            if args.command in ('run', 'upload'):
                route = json.loads(args.json_file.read_text(encoding='utf-8'))
                if args.command == 'upload':
                    print(client.upload(route, args.start_request_id))
                else:
                    print(client.run(route, manual if args.interactive else None, args.start_request_id))
            elif args.command == 'arm-status':
                print(client.arm_status())
            elif args.command == 'arm-zero':
                print(client.arm_zero(args.command_id))
                print(client.wait_arm('READY', command_id=args.command_id, timeout=10))
            elif args.command == 'arm-test-begin':
                from arm_sequence import ArmStation
                station = ArmStation.for_test(client, args.test_id, args.work_id)
                print(station.inspect())
            elif args.command == 'wait-area':
                print(json.dumps(client.wait_start_area()), flush=True)
            elif args.command == 'wait-run':
                def preset(signal):
                    path = args.area1 if signal['area'] == 1 else args.area2
                    return json.loads(path.read_text(encoding='utf-8'))
                print(client.run_after_key(preset, manual if args.interactive else None))
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
