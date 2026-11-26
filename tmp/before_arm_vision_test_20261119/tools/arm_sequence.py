"""Vision integration helpers; no camera algorithm, servo, or tray placement.

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

    def inspect(self):
        arm = self.client.arm_status()
        if tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) != self.context:
            raise RuntimeError('Arm context differs from the selected station')
        self.command_id = max(self.command_id, arm['command_id'])
        return arm

    def ready(self):
        arm = self.client.wait_arm({'WAIT_VISION', 'ALIGNED', 'PICKED'}, self.context)
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

    def lift_after_external_grip_completed(self):
        """No gripper implementation here. Do not call to simulate success."""
        if self.aligned_id is None:
            raise RuntimeError('No aligned object selected')
        command_id = self._next()
        self.client.arm_grip_done(*self.context, command_id, self.aligned_id)
        arm = self.client.wait_arm('PICKED', self.context, command_id)
        self.aligned_id = None
        return arm

    def park(self):
        """Returns READY_LEAVE. Caller then sends existing work_done()."""
        command_id = self._next()
        self.client.arm_finish(*self.context, command_id)
        return self.client.wait_arm('READY_LEAVE', self.context, command_id)
