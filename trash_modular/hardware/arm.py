"""Arm joint control. Publishes raw servo pulse positions - the old project's
ActionGroupController (.d6a action-group playback) was constructed but never
actually used for grasp/drop, so it isn't ported here. If action-group
playback is needed later, add it as its own method rather than reintroducing
an unused dependency.
"""

from servo_controller_msgs.msg import ServoPosition, ServosPosition


class Arm:
    def __init__(self, node, config, logger=None):
        self.node = node
        self.logger = logger or node.get_logger()

        arm_cfg = config.get('arm', {})
        topic = arm_cfg.get('servo_pub_topic', 'servo_controller')
        self.default_duration = float(arm_cfg.get('action_duration_s', 1.5))
        self.poses = {
            name: {int(k): float(v) for k, v in joints.items()}
            for name, joints in arm_cfg.get('poses', {}).items()
        }

        self._pub = node.create_publisher(ServosPosition, topic, 1)
        self.logger.info(f'Arm ready - publishing to {topic}, poses={list(self.poses)}')

    def send(self, joints, duration=None):
        """joints: {servo_id: pulse_position}"""
        msg = ServosPosition()
        msg.duration = float(duration if duration is not None else self.default_duration)
        msg.position_unit = 'pulse'
        for servo_id, position in joints.items():
            servo = ServoPosition()
            servo.id = int(servo_id)
            servo.position = float(position)
            msg.position.append(servo)
        self._pub.publish(msg)
        self.logger.info(f'Arm: sent {joints} over {msg.duration}s')

    def move_to_pose(self, name, duration=None):
        if name not in self.poses:
            self.logger.error(f'Arm: unknown pose "{name}" (known: {list(self.poses)})')
            return False
        self.send(self.poses[name], duration=duration)
        return True

    def go_home(self):
        return self.move_to_pose('home')
