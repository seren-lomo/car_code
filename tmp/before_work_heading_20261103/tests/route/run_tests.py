"""Runs production route.c + route_uart.c with asynchronous chassis/UART mocks."""
import ctypes as C
import json
import math
from pathlib import Path
import random
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import route_client as P

dll = C.CDLL(str(ROOT / 'tests/route/route_test.dll'))
for name in ('test_get', 'test_step', 'test_idle_stale', 'test_robot_step', 'test_request_area'):
    getattr(dll, name).argtypes = [C.c_uint32]
dll.test_float.argtypes = [C.c_uint32]
dll.test_float.restype = C.c_float
dll.test_get.restype = C.c_uint32
dll.test_finish.argtypes = [C.c_float, C.c_float, C.c_float]
dll.test_raw_yaw.argtypes = [C.c_float]
dll.test_control.argtypes = [C.c_uint32, C.c_uint32]
dll.test_key.argtypes = [C.c_uint32, C.c_uint32]
for name in ('test_feed', 'test_dma'):
    getattr(dll, name).argtypes = [C.c_void_p, C.c_uint16]
dll.test_read.argtypes = [C.c_void_p]
dll.test_read.restype = C.c_uint32


def route(points=None, initial=(0, 0, 0), route_id=1):
    def point(x, y, yaw, work=0):
        return dict(x_mm=x, y_mm=y, yaw_deg=yaw, work_id=work)
    return dict(route_id=route_id, initial_pose=point(*initial),
                points=[point(*p) for p in (points or [(500, 0, 0, 1), (500, 300, 90, 2)])])


class RouteTests(unittest.TestCase):
    def setUp(self):
        dll.test_init()
        self.seq = 0

    def step(self, dt=1, count=1):
        for _ in range(count):
            dll.test_step(dt)

    def feed(self, packet, dma=False):
        method = dll.test_dma if dma else dll.test_feed
        method(packet, len(packet))
        self.step(count=5)

    def replies(self):
        self.step(count=35)
        buf = C.create_string_buffer(65536)
        size = dll.test_read(buf)
        messages = P.Decoder().feed(buf.raw[:size])
        return [(kind, seq, P.start_area_payload(body) if kind==P.START_AREA else P.status_payload(body))
                for kind, seq, body in messages]

    def request(self, typ, payload=b'', code=0):
        self.seq += 1
        self.feed(P.frame(typ, self.seq, payload))
        replies = self.replies()
        matching = [status for kind, seq, status in replies
                    if kind == P.ACK and seq == self.seq and status['request'] == typ]
        self.assertEqual(len(matching), 1, replies)
        self.assertEqual(matching[0]['code'], code, matching[0])
        return matching[0]

    def upload(self, r=None, code=0):
        return self.request(P.UPLOAD, P.upload_payload(r or route()), code)

    def start(self, id=1, code=0):
        return self.request(P.START, struct.pack('<I', id), code)

    def drive_to_wait(self, maxlegs=500):
        # An external ideal plant uses Python sin/cos; firmware projects wheel
        # displacement using its own C transform. Completion is asynchronous.
        legs = []
        for _ in range(maxlegs):
            self.step()
            if dll.test_get(0) != 2:
                return legs
            if not dll.test_get(5):
                continue
            action, amount = dll.test_get(4), dll.test_float(0)
            legs.append((action, amount))
            f, l, t = (amount, 0, 0) if action == 2 else ((0, amount, 0) if action == 3 else (0, 0, amount))
            dll.test_finish(f, l, t)
        self.fail('route did not settle')

    def test_absolute_coordinates_and_work_gate(self):
        self.upload()
        self.assertEqual(dll.test_get(3), 0)  # upload cannot start a motor
        self.start()
        first = self.drive_to_wait()
        self.assertEqual(len(first), 1)
        self.assertEqual(first[0][0], 2)
        self.assertAlmostEqual(first[0][1], 500, delta=.02)
        self.assertEqual(dll.test_get(0), 3)
        motions = dll.test_get(3)
        self.step(100, 50)
        self.assertEqual(dll.test_get(3), motions)
        self.request(P.WORK_DONE, struct.pack('<IHBB', 1, 1, 1, 0), 7)
        self.request(P.WORK_DONE, struct.pack('<IHBB', 1, 0, 1, 0))
        second = self.drive_to_wait()
        self.assertEqual(second[0], (1, 90))
        self.assertAlmostEqual(second[1][1], 300, delta=.02)
        self.assertEqual(dll.test_get(1), 1)
        self.request(P.WORK_DONE, struct.pack('<IHBB', 1, 1, 1, 0))
        self.assertEqual(dll.test_get(0), 4)
        self.assertEqual(dll.test_get(10), 2)

    def test_map_projection_all_quadrants_and_chunking(self):
        for yaw in (0, 30, 90, 135, 180, -90, -179):
            with self.subTest(yaw=yaw):
                self.setUp()
                r = route([(8500, -4500, yaw, 0)], initial=(1000, -500, yaw))
                self.upload(r); self.start()
                legs = self.drive_to_wait()
                self.assertEqual(dll.test_get(0), 4)
                self.assertTrue(all(abs(v) <= 4000.001 for a, v in legs if a != 1))
                self.assertAlmostEqual(dll.test_float(1), 8500, delta=6)
                self.assertAlmostEqual(dll.test_float(2), -4500, delta=6)
                # Confirm commanded legs land at the map point using an
                # independent double-precision transformation.
                f = sum(v for a, v in legs if a == 2)
                l = sum(v for a, v in legs if a == 3)
                angle = math.radians(yaw)
                self.assertAlmostEqual(1000 + f*math.cos(angle)-l*math.sin(angle), 8500, delta=6)
                self.assertAlmostEqual(-500 + f*math.sin(angle)+l*math.cos(angle), -4500, delta=6)

    def test_heading_wrap_and_half_turn(self):
        for initial, goal, expected in ((170,-170,20), (-170,170,-20), (0,180,180), (90,-90,180)):
            with self.subTest(initial=initial, goal=goal):
                self.setUp()
                dll.test_raw_yaw(177)
                self.upload(route([(0,0,goal,0)], initial=(0,0,initial)))
                self.start()
                legs = self.drive_to_wait()
                self.assertEqual(legs, [(1, expected)])
                self.assertEqual(dll.test_get(0), 4)

    def test_duplicate_upload_start_and_work_completion(self):
        r = route()
        self.upload(r); self.start()
        n = dll.test_get(3)
        self.upload(r); self.start()
        self.assertEqual(dll.test_get(3), n)
        self.drive_to_wait()
        done = struct.pack('<IHBB',1,0,1,0)
        self.request(P.WORK_DONE, done)
        n = dll.test_get(3)
        self.request(P.WORK_DONE, done)
        self.assertEqual(dll.test_get(3), n)
        self.request(P.WORK_DONE, struct.pack('<IHBB',1,0,0,0), 8)
        self.request(P.STOP, struct.pack('<I',1))
        self.upload(r)
        self.assertEqual(dll.test_get(0), 5)
        self.start(code=7)
        self.upload(route(route_id=2))
        self.upload(r, code=6)

    def test_full_validation_before_commit(self):
        self.upload()
        bad = bytearray(P.upload_payload(route(route_id=2)))
        struct.pack_into('<H', bad, 20+20+12, 121)  # invalid LAST point
        self.request(P.UPLOAD, bad, 4)
        self.assertEqual(dll.test_get(2), 1)
        self.assertEqual(dll.test_get(3), 0)
        r = route(); r['points'][0]['x_mm'] += 1
        self.upload(r, code=8)

    def test_dma_continuous_maximum_route_and_stale_idle(self):
        r = route([(i*10, 0, 0, 0) for i in range(64)])
        packet = P.frame(P.UPLOAD, 22, P.upload_payload(r))
        self.assertEqual(len(packet), 1310)
        self.feed(packet, dma=True)
        self.assertEqual(dll.test_get(0), 1)
        received = dll.test_get(18)
        dll.test_idle_stale(17)  # outdated callback Size must not replay data
        self.step(count=5)
        self.assertEqual(dll.test_get(18), received)
        self.assertGreater(dll.test_get(13), 9)
        self.start(); self.drive_to_wait()
        self.assertEqual(dll.test_get(10), 64)

    def test_noise_crc_fragments_and_partial_timeout(self):
        packet = P.frame(P.UPLOAD, 1, P.upload_payload(route()))
        bad = bytearray(packet); bad[-1] ^= 7
        self.feed(b'noise' + bad)
        self.assertEqual(dll.test_get(0), 0)
        self.assertGreaterEqual(dll.test_get(6), 1)
        for b in packet:
            self.feed(bytes([b]))
        self.assertEqual(dll.test_get(0), 1)
        self.feed(P.frame(P.STATUS,2)[:6])
        self.step(501)
        self.assertGreaterEqual(dll.test_get(8), 1)
        self.request(P.STATUS)

    def test_overflow_and_uart_error_recovery(self):
        self.feed(b'\x77'*2100)
        self.assertEqual(dll.test_get(7), 1)
        self.upload()
        dll.test_control(7, 1)  # fail DMA restart
        self.step(11)
        self.assertGreater(dll.test_get(16), 0)
        dll.test_control(7, 0)
        self.step(11)
        self.feed(P.frame(P.STATUS,8), dma=True)
        self.assertTrue(any(s['request']==P.STATUS for k,q,s in self.replies() if k==P.ACK))

    def test_work_failure_timeout_and_stop(self):
        for mode in ('failure', 'timeout', 'stop'):
            with self.subTest(mode=mode):
                self.setUp()
                if mode == 'timeout': dll.test_control(1, 100)
                self.upload(route([(0,0,0,3)])); self.start()
                self.drive_to_wait()
                if mode == 'failure':
                    self.request(P.WORK_DONE,struct.pack('<IHBB',1,0,0,0))
                    self.assertEqual(dll.test_get(9), 3)
                elif mode == 'timeout':
                    self.step(101)
                    self.assertEqual(dll.test_get(9), 4)
                else:
                    self.request(P.STOP,struct.pack('<I',0))
                    self.assertEqual(dll.test_get(0), 5)
                self.assertNotEqual(dll.test_get(0), 2)

    def test_offline_imu_motor_fault_and_inhibit(self):
        self.upload()
        dll.test_control(2,1); self.start(code=7)
        self.assertEqual(dll.test_get(3),0)
        dll.test_control(2,0); dll.test_control(0,0); self.start(code=7)
        dll.test_control(0,1); self.start()
        dll.test_control(0,0); self.step()
        self.assertEqual(dll.test_get(0),6)
        self.assertEqual(dll.test_get(17),7)
        self.assertFalse(dll.test_get(5))
        self.setUp(); self.upload(); self.start()
        dll.test_control(4,4); self.step()
        self.assertEqual(dll.test_get(9),1)
        self.assertEqual(dll.test_get(17),4)

    def test_vision_context(self):
        body = struct.pack('<IHHii hH',1,0,1,10,-20,100,900)
        self.upload(); self.request(P.VISION,body,7)
        self.start(); self.drive_to_wait()
        self.request(P.VISION,body)
        self.assertEqual(dll.test_get(12),1)
        wrong = struct.pack('<IHHii hH',1,1,1,10,-20,100,900)
        self.request(P.VISION,wrong,7)

    def test_tx_error_and_timeout_do_not_block_executor(self):
        dll.test_control(5,1)
        self.feed(P.frame(P.UPLOAD,1,P.upload_payload(route())))
        self.assertEqual(dll.test_get(0),1)
        self.assertGreater(dll.test_get(14),0)
        dll.test_control(5,0); dll.test_control(6,1)
        self.feed(P.frame(P.STATUS,2))
        self.step(101)
        self.assertGreater(dll.test_get(14),1)
        dll.test_control(6,0)
        self.request(P.STATUS)

    def robot(self, dt=1, count=1):
        for _ in range(count): dll.test_robot_step(dt)

    def press(self, index):
        dll.test_key(index,0); self.robot(); self.robot(30)

    def release(self, index):
        dll.test_key(index,1); self.robot(); self.robot(30)

    def area_ack(self, request_id, area, code=0):
        return self.request(P.AREA_ACK,struct.pack('<IB3x',request_id,area),code)

    def area_upload(self, request_id, r=None, code=0):
        return self.request(P.UPLOAD_FOR_AREA,struct.pack('<I',request_id)+P.upload_payload(r or route()),code)

    def test_keys_select_correct_area_debounce_and_held_once(self):
        for key,area in ((0,2),(1,1)):
            with self.subTest(key=key):
                self.setUp()
                # Bounce before 30 ms must not trigger planning.
                dll.test_key(key,0); self.robot(); self.robot(10)
                dll.test_key(key,1); self.robot(); self.robot(10)
                self.assertEqual(dll.test_get(24),0)
                self.press(key)
                self.assertEqual(dll.test_get(24),1)
                self.assertEqual(dll.test_get(20),area)
                self.robot(100,20)
                self.assertEqual(dll.test_get(24),1)
                self.assertGreater(dll.test_get(23),1)  # retransmission is same request
                signals=[s for k,q,s in self.replies() if k==P.START_AREA]
                self.assertTrue(signals)
                self.assertTrue(all(s['area']==area and s['key']==key and s['request_id']==1 for s in signals))
                self.release(key); self.press(key)
                self.assertEqual(dll.test_get(19),2)

    def test_area_ack_then_token_upload_start_and_work(self):
        self.press(0); self.release(0)
        self.upload(code=7)  # legacy upload cannot bypass selected area token
        self.area_upload(1,code=7)  # must acknowledge area first
        self.area_ack(1,1,code=6)
        self.area_ack(1,2)
        count=dll.test_get(23)
        self.step(600,5)
        self.assertEqual(dll.test_get(23),count)
        self.area_ack(1,2)  # duplicate ACK
        self.area_upload(1)
        self.assertEqual(dll.test_get(22),0)
        self.start(); self.drive_to_wait()
        self.assertEqual(dll.test_get(0),3)
        self.area_upload(1)  # duplicate token upload cannot restart
        self.assertEqual(dll.test_get(0),3)

    def test_area_selection_invalidates_ready_and_old_plan(self):
        self.upload()
        self.press(0); self.release(0)
        self.assertEqual(dll.test_get(0),0)
        self.start(code=6)  # old cached route gone
        self.area_ack(1,2)
        self.press(1); self.release(1)
        self.assertEqual(dll.test_get(19),2)
        self.area_upload(1,route(route_id=2),code=6)
        self.area_ack(1,2,code=6)
        self.area_ack(2,1)
        self.area_upload(2,route(route_id=2))
        self.start(2)
        self.assertEqual(dll.test_get(0),2)

    def test_key0_stops_active_and_does_not_trigger_scan(self):
        for wait in (False,True):
            with self.subTest(wait=wait):
                self.setUp(); self.upload(); self.start()
                if wait: self.drive_to_wait()
                self.press(0)
                self.assertEqual(dll.test_get(0),5)
                self.assertEqual(dll.test_get(24),0)
                self.robot(100,10)
                self.assertEqual(dll.test_get(19),0)
                self.release(0); self.press(0)
                self.assertEqual(dll.test_get(20),2)

    def test_key1_active_and_simultaneous_keys_are_ignored(self):
        self.upload(); self.start()
        self.press(1); self.release(1)
        self.assertEqual(dll.test_get(0),2)
        self.assertEqual(dll.test_get(24),0)
        self.request(P.STOP,struct.pack('<I',0))
        dll.test_key(0,0); dll.test_key(1,0)
        self.robot(); self.robot(35)
        self.assertEqual(dll.test_get(24),0)
        self.release(0); self.robot(50)
        self.assertEqual(dll.test_get(24),0)
        self.release(1); self.robot()
        self.press(1)
        self.assertEqual(dll.test_get(20),1)

    def test_boot_held_key_requires_release(self):
        dll.test_key(0,0); dll.test_keys_init()
        self.robot(100,10)
        self.assertEqual(dll.test_get(24),0)
        self.release(0); self.press(0)
        self.assertEqual(dll.test_get(24),1)

    def test_stop_cancels_pending_area_request(self):
        self.press(1); self.release(1)
        self.request(P.STOP,struct.pack('<I',0))
        count=dll.test_get(23)
        self.step(600,3)
        self.assertEqual(dll.test_get(23),count)
        self.area_ack(1,1,code=7)
        self.area_upload(1,code=7)

    def test_max_area_upload_dma_and_held_start_inhibit(self):
        self.press(1)
        self.area_ack(1,1)
        r=route([(i*10,0,0,0) for i in range(64)])
        packet=P.frame(P.UPLOAD_FOR_AREA,45,struct.pack('<I',1)+P.upload_payload(r))
        self.assertEqual(len(packet),1314)
        self.feed(packet,dma=True)
        self.assertEqual(dll.test_get(0),1)
        self.start(code=7)  # key still held
        self.release(1); self.start(); self.drive_to_wait()
        self.assertEqual(dll.test_get(10),64)


class PythonProtocolTests(unittest.TestCase):
    def test_key_run_waits_for_release_but_respects_cancellation(self):
        client=P.Client(None)
        uploads=[]; attempts=[]
        client.upload=lambda r,token: uploads.append(token)
        def start(route_id):
            attempts.append(route_id)
            if len(attempts)==1:
                raise P.Rejected(dict(code=7,state=1,route_id=route_id))
        client.start=start
        client.status=lambda: dict(route_id=1,state=4,point_index=1,fault=0)
        self.assertEqual(client.run(route([(0,0,0,0)]),start_request_id=12)['state'],4)
        self.assertEqual(uploads,[12])
        self.assertEqual(attempts,[1,1])
        def cancelled(route_id):
            raise P.Rejected(dict(code=7,state=5,route_id=route_id))
        client.start=cancelled
        with self.assertRaises(P.Rejected):
            client.run(route(),start_request_id=12)

    def test_wait_key_acks_then_plans_and_uses_token(self):
        class FakeSerial:
            def __init__(self):
                signal=struct.pack('<IIBBH',17,1234,2,0,0)
                self.data=P.frame(P.START_AREA,90,signal)*2
                self.types=[]
            @property
            def in_waiting(self): return len(self.data)
            def read(self,n): result,self.data=self.data[:n],self.data[n:]; return result
            def write(self,p):
                typ,seq,body=P.Decoder().feed(p)[0]
                self.types.append(typ)
                if typ==P.AREA_ACK: self.assert_payload=struct.unpack('<IB3x',body)
                status=struct.pack('<BBBBIHHHBBIIiihH',typ,0,1,0,1,0,2,0,0,0,10,0,0,0,0,0)
                self.data+=P.frame(P.ACK,seq,status)
                return len(p)
        serial=FakeSerial(); client=P.Client(serial)
        planned=[]
        def planner(signal):
            self.assertEqual(serial.types,[P.AREA_ACK])
            self.assertEqual(signal['area'],2)
            self.assertEqual(serial.assert_payload,(17,2))
            planned.append(signal)
            return route()
        def fake_run(r,on_work=None,start_request_id=None):
            self.assertEqual(start_request_id,17)
            client.upload(r,start_request_id)
            return 'accepted'
        client.run=fake_run
        self.assertEqual(client.run_after_key(planner),'accepted')
        self.assertEqual(serial.types,[P.AREA_ACK,P.UPLOAD_FOR_AREA])
        with self.assertRaises(TimeoutError): client.wait_start_area(timeout=.001)
        self.assertEqual(len(planned),1)

    def test_crc_and_decoder(self):
        self.assertEqual(P.crc16(b'123456789'), 0x29b1)
        rng = random.Random(24)
        packets = [P.frame(P.STATUS,i) for i in range(100)]
        data = b'junk' + b''.join(packets)
        decoder = P.Decoder(); replies=[]
        while data:
            n=rng.randrange(1,20); replies.extend(decoder.feed(data[:n])); data=data[n:]
        self.assertEqual([seq for typ,seq,body in replies],list(range(100)))

    def test_example_and_validation(self):
        example=json.loads((ROOT/'tools/route_demo.json').read_text())
        self.assertEqual(len(P.upload_payload(example)),100)
        for key, value in (('yaw_deg',float('nan')), ('x_mm',100001)):
            bad=route(); bad['points'][0][key]=value
            with self.assertRaises(ValueError): P.upload_payload(bad)

    def test_client_event_before_ack_and_retry(self):
        class FakeSerial:
            def __init__(self): self.data=b''; self.writes=[]
            @property
            def in_waiting(self): return len(self.data)
            def read(self,n):
                result,self.data=self.data[:n],self.data[n:]; return result
            def write(self,p):
                self.writes.append(p)
                if len(self.writes)==2:
                    # A legitimate event comes before the ACK on a retry.
                    status=struct.pack('<BBBBIHHHBBIIiihH',P.STATUS,0,1,0,1,0,2,0,0,0,10,0,0,0,0,0)
                    self.data=P.frame(P.EVENT,77,status)+P.frame(P.ACK,1,status)
                return len(p)
        serial=FakeSerial(); client=P.Client(serial)
        self.assertEqual(client.request(P.STATUS,timeout=.001)['route_id'],1)
        self.assertEqual(serial.writes[0],serial.writes[1])
        self.assertEqual(len(client.events),1)


if __name__=='__main__':
    unittest.main(verbosity=2)
