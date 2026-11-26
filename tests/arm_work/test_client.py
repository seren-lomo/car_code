"""UART4 arm protocol/client behavior without hardware."""
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import route_client as P
from arm_sequence import ArmStation, MotorPose


def arm_body(state=6, command=10, aligned=10, context=(99, 2, 102), test_mode=False):
    return struct.pack('<IHHIIBBBBHHiiiiiiI', *context, command, aligned,
                       state, 0, 255, 2, 5 | (8 if test_mode else 0), 0, 1000, -2000, 3000,
                       1000, -2000, 3000, 1234)


def route_body(request, context=(99, 2, 102), state=3):
    return struct.pack('<BBBBIHHHBBIIiihH', request, 0, state, 4,
                       context[0], context[1], 20, context[2], 0, 0,
                       1234, 0, 350, 1200, 18000, 0)


class FakeSerial:
    def __init__(self, include_arm=True, drop_first=False):
        self.data = b''
        self.writes = []
        self.include_arm = include_arm
        self.drop_first = drop_first
        self.arm_state = 6
        self.route_state = 3
        self.command = 10
        self.aligned = 10
        self.actions = []
        self.test_mode = False
        self.context = (99, 2, 102)

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, n):
        data, self.data = self.data[:n], self.data[n:]
        return data

    def write(self, packet):
        self.writes.append(packet)
        if self.drop_first and len(self.writes) == 1:
            return len(packet)
        kind, sequence, body = P.Decoder().feed(packet)[0]
        if kind == P.ARM_COMMAND:
            rid, index, work, self.command, opcode, mode, _, _, _, _, ref = struct.unpack('<IHHIBBHiiiI', body)
            if opcode == 6:
                self.context = (rid, index, work)
                self.aligned = 0
            self.actions.append(opcode)
            if opcode in (2, 10):
                self.aligned = self.command
            if opcode in (3, 4, 8, 9):
                assert ref == self.aligned
            self.arm_state = {1: 2, 2: 6, 3: 8, 4: 10, 5: 12, 6: 4, 7: 19, 8: 8, 9: 4,
                              10: 9 if mode else 8}[opcode]
        if kind in (P.ARM_STATUS, P.ARM_COMMAND) and self.include_arm:
            self.data += P.frame(P.ARM_EVENT, 77, arm_body(
                self.arm_state, self.command, self.aligned, self.context, self.test_mode))
        self.data += P.frame(P.ACK, sequence, route_body(kind, context=self.context, state=self.route_state))
        return len(packet)


class ClientTests(unittest.TestCase):
    def test_align_pick_wire_modes_and_abort_reference(self):
        serial = FakeSerial()
        client = P.Client(serial)
        client.arm_align_pick(99, 2, 102, 10, -440, 20, -120)
        self.assertEqual(serial.arm_state, 8)
        client.arm_grip_abort(99, 2, 102, 11, 10)
        self.assertEqual(serial.arm_state, 4)
        client.arm_align_pick(99, 2, 102, 12, -440, 20, -120, auto_load=True)
        self.assertEqual(serial.arm_state, 9)
        bodies = [P.Decoder().feed(packet)[0][2] for packet in serial.writes]
        self.assertEqual([len(body) for body in bodies], [32, 32, 32])
        self.assertEqual([body[12:16] for body in bodies],
                         [bytes((10, 0, 0, 0)), bytes((9, 0, 0, 0)), bytes((10, 1, 0, 0))])
        self.assertEqual(struct.unpack_from('<iiiI', bodies[0], 16), (-44000, 2000, -12000, 0))
        self.assertEqual(struct.unpack_from('<I', bodies[1], 28)[0], 10)
        with self.assertRaises(ValueError):
            P.arm_command_payload(99, 2, 102, 13, 10, reference_id=12)

    def test_standalone_session_without_a_running_route(self):
        serial = FakeSerial()
        serial.test_mode = True
        serial.route_state = 0
        client = P.Client(serial)
        station = ArmStation.for_test(client, 1, 102)
        station.align(MotorPose(-10, 20, -30))
        station.lower_after_visual_confirmation()
        self.assertEqual(serial.arm_state, 8)
        station.lift_after_external_grip_completed()
        station.park()
        self.assertEqual(serial.actions, [6, 2, 3, 4, 5])
        self.assertEqual(station.context, (1, 0, 102))
        types = [P.Decoder().feed(p)[0][0] for p in serial.writes]
        self.assertNotIn(P.START, types)
        self.assertNotIn(P.STATUS, types)
        self.assertNotIn(P.WORK_DONE, types)

    def test_standalone_requires_test_firmware(self):
        with self.assertRaises(RuntimeError):
            ArmStation.for_test(P.Client(FakeSerial()), 1, 102)
        self.assertFalse(P.arm_status_payload(arm_body())['test_mode'])

    def test_standalone_stops_on_a_changed_test_context(self):
        serial = FakeSerial()
        serial.test_mode = True
        with self.assertRaises(RuntimeError):
            P.Client(serial).wait_arm('ALIGNED', (1, 0, 102), timeout=0.1)

    def test_standalone_begin_layout_and_work_validation(self):
        serial = FakeSerial()
        client = P.Client(serial)
        client.arm_test_begin(5, 107)
        kind, _, body = P.Decoder().feed(serial.writes[0])[0]
        self.assertEqual(kind, P.ARM_COMMAND)
        self.assertEqual(struct.unpack('<IHHIBBHiiiI', body), (5, 0, 107, 1, 6, 0, 0, 0, 0, 0, 0))
        for session, work in ((0, 102), (1, 101), (1, 108)):
            with self.assertRaises(ValueError):
                client.arm_test_begin(session, work)

    def test_absolute_motor_angles_and_reference_layout(self):
        body = P.arm_command_payload(99, 2, 102, 10, 2, 1.5, -2, 360)
        self.assertEqual(len(body), 32)
        self.assertEqual(struct.unpack('<IHHIBBHiiiI', body), (99, 2, 102, 10, 2, 0, 0, 150, -200, 36000, 0))
        self.assertEqual(struct.unpack_from('<I', P.arm_command_payload(99, 2, 102, 11, 3, reference_id=10), 28)[0], 10)

    def test_vision_slot_and_release_pose_layout(self):
        serial = FakeSerial()
        serial.context = (99, 2, 103)
        station = ArmStation(P.Client(serial), P.status_payload(route_body(0x14, context=serial.context)))
        station.ready()
        arm = station.place_next_from_tray(3, MotorPose(-10, 20, -30))
        kind, _, body = next(P.Decoder().feed(packet)[0] for packet in serial.writes
                             if packet[3] == P.ARM_COMMAND)
        self.assertEqual(kind, P.ARM_COMMAND)
        self.assertEqual(struct.unpack('<IHHIBBHiiiI', body), (99, 2, 103, 11, 7, 3, 0, -1000, 2000, -3000, 0))
        self.assertEqual(arm['state_name'], 'PLACED')
        for slot in (0, 4, True):
            with self.assertRaises(ValueError):
                P.arm_command_payload(99, 2, 103, 11, 7, destination_slot=slot)

    def test_rough_processing_reload_layout(self):
        serial = FakeSerial()
        serial.context = (99, 2, 103)
        station = ArmStation(P.Client(serial), P.status_payload(route_body(0x14, context=serial.context)))
        station.ready()
        station.align(MotorPose(0, 20, -30))
        arm = station.lower_for_tray_reload(2, -10)
        commands = [P.Decoder().feed(packet)[0][2] for packet in serial.writes
                    if packet[3] == P.ARM_COMMAND]
        self.assertEqual(struct.unpack('<IHHIBBHiiiI', commands[-1]),
                         (99, 2, 103, 12, 8, 2, 0, -1000, 0, 0, 11))
        self.assertEqual(arm['state_name'], 'WAIT_GRIP')

    def test_arm_motor_fault_report_fields(self):
        body = bytearray(arm_body(state=14))
        struct.pack_into('<H', body, 20, 28 | (5 << 6) | (2 << 9) | (1 << 13) | (0x0c << 12))
        arm = P.arm_status_payload(body)
        self.assertEqual((arm['motor_fault'], arm['motor_fault_stage'], arm['motor_fault_flags']), (5, 2, 0x0d))
        self.assertTrue(arm['test_mode'] and arm['tray_enabled'])
        self.assertEqual(P.arm_status_payload(arm_body())['motor_fault'], 0)

    def test_tray_report_fields(self):
        body = bytearray(arm_body(state=19))
        struct.pack_into('<HBB', body, 20, 1 | 16 | 32, 2, 3)
        arm = P.arm_status_payload(body)
        self.assertTrue(arm['tray_enabled'] and arm['tray_uncertain'])
        self.assertEqual((arm['tray_count'], arm['destination_slot']), (2, 3))

    def test_invalid_pose_and_bad_report_length(self):
        for value in (float('nan'), float('inf'), 30000000, True):
            with self.assertRaises(ValueError):
                P.arm_command_payload(99, 2, 102, 10, 2, yaw_deg=value)
        with self.assertRaises(ValueError):
            P.arm_status_payload(b'bad')

    def test_arm_report_is_distinct_from_route_heading(self):
        report = P.arm_status_payload(arm_body())
        self.assertEqual(report['yaw_deg'], -20)
        self.assertEqual(report['state_name'], 'ALIGNED')
        self.assertEqual(P.status_payload(route_body(0x32))['yaw_deg'], 180)

    def test_interleaved_arm_event_before_matching_ack(self):
        serial = FakeSerial()
        client = P.Client(serial)
        self.assertEqual(client.arm_status()['state_name'], 'ALIGNED')
        self.assertEqual(len(client.arm_events), 1)
        self.assertEqual(client.status()['state'], 3)
        self.assertEqual(client.arm_generation, 1)

    def test_retry_keeps_exact_motion_id_and_frame(self):
        serial = FakeSerial(drop_first=True)
        client = P.Client(serial)
        payload = P.arm_command_payload(99, 2, 102, 11, 3, reference_id=10)
        client.request(P.ARM_COMMAND, payload, timeout=0.001)
        self.assertEqual(serial.writes[0], serial.writes[1])
        self.assertEqual(serial.actions, [3])

    def test_missing_report_is_not_a_motion_success(self):
        with self.assertRaises(TimeoutError):
            P.Client(FakeSerial(include_arm=False)).arm_status()

    def test_wait_stops_on_fault_or_context_change(self):
        serial = FakeSerial()
        serial.arm_state = 14
        with self.assertRaises(RuntimeError):
            P.Client(serial).wait_arm('ALIGNED', (99, 2, 102), timeout=0.1)
        serial = FakeSerial()
        serial.route_state = 5
        with self.assertRaises(RuntimeError):
            P.Client(serial).wait_arm('ALIGNED', (99, 2, 102), timeout=0.1)

    def test_station_requires_explicit_external_grip_confirmation(self):
        serial = FakeSerial()
        client = P.Client(serial)
        station = ArmStation(client, P.status_payload(route_body(0x14)))
        station.ready()
        station.align(MotorPose(0, 5, 20))
        station.lower_after_visual_confirmation()
        self.assertEqual(serial.actions, [2, 3])
        self.assertEqual(serial.arm_state, 8)
        station.lift_after_external_grip_completed()
        station.park()
        self.assertEqual(serial.actions, [2, 3, 4, 5])
        self.assertEqual(station.command_id, 14)


if __name__ == '__main__':
    unittest.main(verbosity=2)
