"""ros2 run trash_modular calibrate_gripper -- --label empty
ros2 run trash_modular calibrate_gripper -- --label grasped

Logs the gripper servo's commanded-vs-actual position to a CSV so you can
compare "closed on nothing" vs "closed on an object" position distributions,
and pick gripper.resist_threshold in config.yaml accordingly.

This node only OBSERVES - trigger the gripper close itself with test_gripper
in another terminal:
    ros2 run trash_modular test_gripper -- --close
"""

import argparse
import csv
import os
import sys
import time

import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServoStateList

from trash_modular.config.params import load_config

CSV_PATH = 'gripper_calibration_log.csv'
SETTLE_S = 1.2


class GripperCalibrationLogger(Node):
    def __init__(self, label, servo_id, csv_path):
        super().__init__('calibrate_gripper')
        self.label = label
        self.servo_id = servo_id
        self.csv_path = csv_path
        self._last_goal = None
        self._settle_start = None
        self._settled = False

        if not os.path.isfile(csv_path):
            with open(csv_path, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'label', 'goal', 'position', 'deficit'])

        gripper_cfg = load_config().get('gripper', {})
        topic = gripper_cfg.get('servo_states_topic', '/controller_manager/servo_states')
        self.create_subscription(ServoStateList, topic, self._callback, 10)
        self.get_logger().info(f'Logging servo {servo_id} feedback with label="{label}" -> {csv_path}')

    def _callback(self, msg):
        for st in msg.servo_state:
            if st.id != self.servo_id:
                continue
            if st.goal != self._last_goal:
                self._last_goal = st.goal
                self._settle_start = time.time()
                self._settled = False

            if not self._settled and self._settle_start and (time.time() - self._settle_start) > SETTLE_S:
                self._settled = True
                deficit = st.goal - st.position
                with open(self.csv_path, 'a', newline='') as f:
                    csv.writer(f).writerow([time.time(), self.label, st.goal, st.position, deficit])
                self.get_logger().info(f'[{self.label}] goal={st.goal} position={st.position} deficit={deficit}')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', required=True, help='e.g. "empty" or "grasped"')
    parser.add_argument('--csv', default=CSV_PATH)
    ns, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    config = load_config()
    servo_id = config.get('gripper', {}).get('servo_id', 10)
    node = GripperCalibrationLogger(ns.label, servo_id, ns.csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
