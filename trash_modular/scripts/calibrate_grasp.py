"""ros2 run trash_modular calibrate_grasp

Interactive tool to collect real grasp-pose samples for
manipulation.grasp.GraspCalibrator (config arm.grasp.calibration_csv).
Without samples, GraspCalibrator.predict() always falls back to the fixed
baseline pose (baseline_j1/baseline_lift/baseline_reach) no matter what the
object's actual pixel position or measured depth is - this is the tool that
was missing to make reach/lift/joint1 actually adapt to depth instead of
staying constant every grasp.

WORKFLOW
--------
1. Place a real object where the robot would normally stop to grasp it
   (roughly navigation.approach_lidar_distance_m in front, centred).
2. Run this tool. It detects the object with the same detector the pipeline
   uses, reads its depth, and sends the arm to the baseline grasp pose.
3. Jog the arm with the keys below until the gripper is positioned to
   actually grasp the object correctly - watch the real robot, not the
   pixel/depth numbers.
4. Optionally press SPACE to test-close the gripper on the object and see
   the grasp-verification deficit, then it reopens automatically.
5. Press ENTER to save this as a sample (px, py, z3d, and the pulse DELTA
   from baseline for each joint) to grasp_calibration.csv, then it looks for
   the next object. Collect samples spanning the pixel/depth range you
   actually see at grasp time - GraspCalibrator only trusts nearby samples
   (arm.grasp.max_trusted_distance_px) and falls back to baseline otherwise.

KEYS
----
  a / d      - joint1 (turret) left / right
  w / s      - lift up / down
  q / e      - reach in / out
  space      - test: close gripper, report grasp verification, reopen
  ENTER      - save this sample, then look for the next object
  n          - skip this object without saving, look for the next one
  Ctrl+C     - quit
"""

import csv
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config, resolve_path
from trash_modular.hardware.arm import Arm
from trash_modular.hardware.gripper import Gripper
from trash_modular.perception.camera import Camera
from trash_modular.perception.depth import DepthSensor
from trash_modular.perception.object_detector import create_detector

JOG_STEP_PULSES = 4


def read_key_nonblocking(fd, timeout=0.05):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def wait_for_detection(node, camera, detector, confidence_threshold, fd):
    print('\nLooking for an object to calibrate against (Ctrl+C to quit)...')
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if read_key_nonblocking(fd, timeout=0.0) == '\x03':
            return None
        frame = camera.get_frame()
        if frame is None:
            continue
        detection = detector.detect(frame)
        if detection.visible and detection.confidence >= confidence_threshold:
            return detection
    return None


def jog_to_grasp_pose(node, arm, gripper, fd, j1, lift, reach):
    """Returns (saved: bool, j1, lift, reach)."""
    print(
        "\nJog to the correct grasp pose:\n"
        "  a/d = joint1 (turret)   w/s = lift   q/e = reach\n"
        "  space = test-close gripper on it   ENTER = save sample\n"
        "  n = skip this object   Ctrl+C = quit"
    )
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        key = read_key_nonblocking(fd, timeout=0.05)
        if key is None:
            continue

        moved = False
        if key == 'a':
            j1 -= JOG_STEP_PULSES
            moved = True
        elif key == 'd':
            j1 += JOG_STEP_PULSES
            moved = True
        elif key == 'w':
            lift += JOG_STEP_PULSES
            moved = True
        elif key == 's':
            lift -= JOG_STEP_PULSES
            moved = True
        elif key == 'q':
            reach -= JOG_STEP_PULSES
            moved = True
        elif key == 'e':
            reach += JOG_STEP_PULSES
            moved = True
        elif key == ' ':
            gripper.close()
            time.sleep(1.2)
            grasped, deficit = gripper.is_grasped()
            print(f'\n  test-close: grasped={grasped} deficit={deficit}')
            gripper.open()
            time.sleep(0.8)
        elif key in ('\n', '\r'):
            return True, j1, lift, reach
        elif key == 'n':
            return False, j1, lift, reach
        elif key == '\x03':
            return False, j1, lift, reach

        if moved:
            arm.send({1: j1, 2: lift, 3: reach}, duration=0.3)
            print(f'\r  j1={j1:.0f} lift={lift:.0f} reach={reach:.0f}   ', end='', flush=True)

    return False, j1, lift, reach


def save_sample(csv_path, px, py, z3d, delta_j1, delta_lift, delta_reach):
    write_header = not os.path.isfile(csv_path)
    os.makedirs(os.path.dirname(csv_path) or '.', exist_ok=True)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(['px', 'py', 'z3d', 'delta_j1', 'delta_lift', 'delta_reach'])
        writer.writerow([px, py, round(z3d, 4), round(delta_j1, 1), round(delta_lift, 1), round(delta_reach, 1)])


def main(args=None):
    rclpy.init(args=args)
    node = Node('calibrate_grasp')
    log = node.get_logger()

    config = load_config()
    camera = Camera(node, config, log)
    depth = DepthSensor(node, config, log)
    arm = Arm(node, config, log)
    gripper = Gripper(node, arm, config, log)
    detector = create_detector(config, log)
    confidence_threshold = config.get('detection', {}).get('confidence_threshold', 0.6)

    grasp_cfg = config.get('arm', {}).get('grasp', {})
    baseline_j1 = float(arm.poses.get('home', {}).get(1, 500))
    baseline_lift = float(grasp_cfg.get('baseline_lift', 350))
    baseline_reach = float(grasp_cfg.get('baseline_reach', 215))
    csv_path = resolve_path(config, grasp_cfg.get('calibration_csv', 'config/grasp_calibration.csv'))

    print('calibrate_grasp ready.')
    print(f'Baseline pose: j1={baseline_j1:.0f} lift={baseline_lift:.0f} reach={baseline_reach:.0f}')
    print(f'Samples will be appended to: {csv_path}')

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while rclpy.ok():
            detection = wait_for_detection(node, camera, detector, confidence_threshold, fd)
            if detection is None:
                break

            px, py = detection.center_x, detection.center_y
            print(f'Detected {detection.material} at pixel=({px},{py}) conf={detection.confidence:.2f}')

            point = depth.pixel_to_point(px, py)
            if point is None:
                print('No valid depth at this pixel - skipping. Reposition and try again.')
                time.sleep(1.0)
                continue
            z3d = point[2]
            print(f'Depth z={z3d:.3f}m')

            arm.send({1: baseline_j1, 2: baseline_lift, 3: baseline_reach, 4: 250, 5: 500}, duration=1.5)
            time.sleep(1.6)

            saved, j1, lift, reach = jog_to_grasp_pose(node, arm, gripper, fd, baseline_j1, baseline_lift, baseline_reach)
            if saved:
                delta_j1 = j1 - baseline_j1
                delta_lift = lift - baseline_lift
                delta_reach = reach - baseline_reach
                save_sample(csv_path, px, py, z3d, delta_j1, delta_lift, delta_reach)
                print(
                    f'\nSaved: px={px} py={py} z3d={z3d:.3f} '
                    f'delta=({delta_j1:+.0f},{delta_lift:+.0f},{delta_reach:+.0f}) -> {csv_path}'
                )

            arm.go_home()
            time.sleep(1.5)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
