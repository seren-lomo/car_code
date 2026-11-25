"""Vision integration helpers for STM32 arm and tray commands.

Pass the existing single route_client.Client. Never open another UART reader.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class MotorPose:
    z_deg: float
    yaw_deg: float
    reach_deg: float


class ArmStation:
    """One controller WAIT_WORK context, command IDs increase at this station."""
    def __init__(self, client, route_status):
        self.client = client
        self.context = tuple(route_status[k] for k in ('route_id', 'point_index', 'work_id'))
        if route_status['state'] != 3 or self.context[2] not in range(102, 108):
            raise ValueError('ArmStation requires material/processing/storage WAIT_WORK')
        # Recover command numbering after a Pi reconnect without repeating picks.
        # Call inspect() first; only continue if the real mechanism matches the report.
        self.command_id = 0
        self.aligned_id = None

    @classmethod
    def for_test(cls, client, test_id, work_id):
        """Enter a work area without a route; manual origins must be ready.

        Each new session needs a larger test_id, including after STOP/re-zero.
        Same-session retries inspect current state and never repeat a pick.
        """
        arm = client.arm_status()
        if not arm['test_mode']:
            raise RuntimeError('Rebuild STM32 with ARM_VISION_TEST_MODE=1 first')
        client.arm_test_begin(test_id, work_id)
        station = cls(client, dict(state=3, route_id=test_id, point_index=0, work_id=work_id))
        station.ready()
        return station

    def inspect(self):
        arm = self.client.arm_status()
        if tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) != self.context:
            raise RuntimeError('Arm context differs from the selected station')
        self.command_id = max(self.command_id, arm['command_id'])
        return arm

    def ready(self):
        arm = self.client.wait_arm({'WAIT_VISION', 'ALIGNED', 'PICKED', 'PLACED'}, self.context)
        self.command_id = max(self.command_id, arm['command_id'])
        return arm

    def _next(self):
        self.command_id += 1
        if self.command_id > 0xffffffff:
            raise OverflowError('command ID exhausted; stop and select a new route')
        return self.command_id

    def align(self, pose):
        """Call again for each camera correction; every pose is absolute."""
        command_id = self._next()
        self.client.arm_align(*self.context, command_id, pose.z_deg, pose.yaw_deg, pose.reach_deg)
        arm = self.client.wait_arm('ALIGNED', self.context, command_id)
        self.aligned_id = arm['aligned_command_id']
        return arm

    def lower_after_visual_confirmation(self):
        """Caller must confirm image alignment AFTER the last align() finished."""
        if self.aligned_id is None:
            raise RuntimeError('Call align() and verify the image first')
        command_id = self._next()
        self.client.arm_pick(*self.context, command_id, self.aligned_id)
        return self.client.wait_arm('WAIT_GRIP', self.context, command_id)

    def lower_for_tray_reload(self, slot, pick_z_deg):
        """After alignment, use the measured absolute pick Z at the processing slot."""
        if self.aligned_id is None:
            raise RuntimeError('Call align() and verify the image first')
        command_id = self._next()
        self.client.arm_tray_reload_pick(*self.context, command_id, slot, pick_z_deg, self.aligned_id)
        return self.client.wait_arm('WAIT_GRIP', self.context, command_id)

    def lift_after_external_grip_completed(self):
        """Confirm grip and wait for PICKED, including tray loading when enabled."""
        if self.aligned_id is None:
            raise RuntimeError('No aligned object selected')
        command_id = self._next()
        self.client.arm_grip_done(*self.context, command_id, self.aligned_id)
        arm = self.client.wait_arm('PICKED', self.context, command_id)
        self.aligned_id = None
        return arm

    def place_next_from_tray(self, slot, pose):
        """Use vision's slot number and measured release pose for the next item."""
        command_id = self._next()
        self.client.arm_tray_place(*self.context, command_id, slot, pose.z_deg,
                                  pose.yaw_deg, pose.reach_deg)
        return self.client.wait_arm('PLACED', self.context, command_id)

    def park(self):
        """Returns READY_LEAVE. Only the vehicle workflow then sends work_done()."""
        command_id = self._next()
        self.client.arm_finish(*self.context, command_id)
        return self.client.wait_arm('READY_LEAVE', self.context, command_id)
