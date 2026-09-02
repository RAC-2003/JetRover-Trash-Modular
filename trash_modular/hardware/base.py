"""Base (drivetrain) motion primitive. Every velocity command in this project
goes through this one class so speed clamping and the watchdog are enforced
in exactly one place - the old project published Twist directly from many
call sites with no shared safety layer.

This module only knows how to set/hold/stop a velocity. It does not do
rotate-by-angle or go-to-pose - see navigation/movement.py and
navigation/position.py, which are built on top of this.
"""

from geometry_msgs.msg import Twist

from trash_modular.utils.watchdog import CommandWatchdog


class RobotBase:
    def __init__(self, node, config, logger=None):
        self.node = node
        self.logger = logger or node.get_logger()

        topic = config.get('robot', {}).get('cmd_vel_topic', '/controller/cmd_vel')
        self.max_linear = float(config.get('robot', {}).get('max_linear_speed', 0.15))
        self.max_angular = float(config.get('robot', {}).get('max_angular_speed', 0.6))
        watchdog_timeout = float(config.get('robot', {}).get('watchdog_timeout_s', 0.5))

        self._pub = node.create_publisher(Twist, topic, 10)
        self._watchdog = CommandWatchdog(watchdog_timeout)
        self._linear = 0.0
        self._angular = 0.0
        self._timer = node.create_timer(0.05, self._publish_tick)

        self.logger.info(f'RobotBase ready - publishing to {topic}')

    def move(self, linear=0.0, angular=0.0):
        self._linear = max(-self.max_linear, min(self.max_linear, linear))
        self._angular = max(-self.max_angular, min(self.max_angular, angular))
        self._watchdog.refresh()
        self._publish(self._linear, self._angular)

    def stop(self):
        self._linear = 0.0
        self._angular = 0.0
        self._watchdog.refresh()
        for _ in range(3):
            self._publish(0.0, 0.0)

    def _publish_tick(self):
        if self._watchdog.expired():
            self._linear = 0.0
            self._angular = 0.0
        self._publish(self._linear, self._angular)

    def _publish(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._pub.publish(msg)
