"""Gripper open/close + grasp verification via servo feedback.

Verification uses the "deficit" between the commanded close position and the
servo's actual settled position: a gripper closing on something can't reach
its full closed target, a gripper closing on empty air can. The old project
flagged resist_threshold as uncalibrated - it stays a config value here
(gripper.resist_threshold) so it can be tuned with scripts/calibrate_gripper.py
without touching code.
"""

from servo_controller_msgs.msg import ServoStateList


class Gripper:
    def __init__(self, node, arm, config, logger=None):
        self.node = node
        self.arm = arm
        self.logger = logger or node.get_logger()

        gripper_cfg = config.get('gripper', {})
        self.servo_id = int(gripper_cfg.get('servo_id', 10))
        self.open_position = float(gripper_cfg.get('open_position', 100))
        self.close_position = float(gripper_cfg.get('close_position', 700))
        self.resist_threshold = float(gripper_cfg.get('resist_threshold', 60))
        self.settle_time_s = float(gripper_cfg.get('settle_time_s', 1.0))

        self._servo_states = {}
        topic = gripper_cfg.get('servo_states_topic', '/controller_manager/servo_states')
        node.create_subscription(ServoStateList, topic, self._servo_states_cb, 10)
        self.logger.info(f'Gripper ready - servo_id={self.servo_id}, feedback topic={topic}')

    def _servo_states_cb(self, msg):
        for st in msg.servo_state:
            self._servo_states[st.id] = st

    def open(self, duration=0.5):
        self.arm.send({self.servo_id: self.open_position}, duration=duration)
        self.logger.info('Gripper: open')

    def close(self, duration=0.5):
        self.arm.send({self.servo_id: self.close_position}, duration=duration)
        self.logger.info('Gripper: close')

    def last_state(self):
        return self._servo_states.get(self.servo_id)

    def is_grasped(self):
        """Returns (grasped: bool, deficit: float or None)."""
        st = self.last_state()
        if st is None:
            self.logger.warn(f'Gripper: no servo feedback for id={self.servo_id}')
            return False, None
        deficit = self.close_position - float(st.position)
        grasped = deficit >= self.resist_threshold
        self.logger.info(
            f'Gripper: position={st.position} target={self.close_position} '
            f'deficit={deficit:.0f} threshold={self.resist_threshold} -> '
            f'{"grasped" if grasped else "empty"}'
        )
        return grasped, deficit
