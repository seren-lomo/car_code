"""UART4 arm protocol/client behavior without hardware."""
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import route_client as P
from arm_sequence import ArmStation, MotorPose


def arm_body(state=6, command=10, aligned=10, context=(99, 2, 102)):
    return struct.pack('<IHHIIBBBBHHiiiiiiI', *context, command, aligned,
                       state, 0, 255, 2, 5, 0, 1000, -2000, 3000,
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
            _, _, _, self.command, opcode, _, _, _, _, _, ref = struct.unpack('<IHHIBBHiiiI', body)
            self.actions.append(opcode)
            if opcode == 2:
                self.aligned = self.command
            if opcode in (3, 4):
                assert ref == self.aligned
            self.arm_state = {1: 2, 2: 6, 3: 8, 4: 10, 5: 12}[opcode]
        if kind in (P.ARM_STATUS, P.ARM_COMMAND) and self.include_arm:
            self.data += P.frame(P.ARM_EVENT, 77, arm_body(self.arm_state, self.command, self.aligned))
        self.data += P.frame(P.ACK, sequence, route_body(kind, state=self.route_state))
        return len(packet)


class ClientTests(unittest.TestCase):
    def test_absolute_motor_angles_and_reference_layout(self):
        body = P.arm_command_payload(99, 2, 102, 10, 2, 1.5, -2, 360)
        self.assertEqual(len(body), 32)
        self.assertEqual(struct.unpack('<IHHIBBHiiiI', body), (99, 2, 102, 10, 2, 0, 0, 150, -200, 36000, 0))
        self.assertEqual(struct.unpack_from('<I', P.arm_command_payload(99, 2, 102, 11, 3, reference_id=10), 28)[0], 10)

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
