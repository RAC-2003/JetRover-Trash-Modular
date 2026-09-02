"""ros2 run trash_modular calibrate_home_bin -- --record --material recyclable
ros2 run trash_modular calibrate_home_bin -- --test --material recyclable

Records a bin's position as an offset from HOME (not an absolute odom
coordinate) because /odom re-zeros every time its driver restarts - an
absolute coordinate recorded in one session means nothing in the next. This
is the fix the old project's home_to_bin.py had but which never made it into
trash_sorter.py itself; here there's only one implementation, used by both
calibration and the pipeline (navigation.position.resolve_offset_target).

--record: prompts you to place the robot at HOME, then jog it (w/a/s/d) to
the bin, and saves the offset to bins.calibration_file plus prints a
config.yaml snippet.
--test: prompts you to place the robot at HOME, then drives autonomously to
the saved offset using the same go_to_pose the pipeline uses.
"""

import argparse
import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config, resolve_path
from trash_modular.hardware.base import RobotBase
from trash_modular.navigation.position import PositionTracker, resolve_offset_target
from trash_modular.perception.imu import ImuSensor
from trash_modular.utils.transforms import normalize_deg

JOG_LINEAR_SPEED = 0.1
JOG_ANGULAR_SPEED = 0.4
JOG_KEY_TIMEOUT_S = 0.3  # held-key stops within this long after release


def jog_until_enter(node, base, position, prompt, reference_pose=None):
    """Interactive w/a/s/d jog control (held-key style - keeps moving while
    the key repeats, stops shortly after release) until ENTER is pressed.
    space also stops immediately. If reference_pose is given, prints live
    distance from it while jogging."""
    print(prompt)
    print('  w=forward  s=backward  a=turn-left  d=turn-right  space=stop  ENTER=done')

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    jog_linear, jog_angular = 0.0, 0.0
    last_key_time = 0.0
    last_status = 0.0

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)

            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch in ('\n', '\r'):
                    break
                elif ch == 'w':
                    jog_linear, jog_angular = JOG_LINEAR_SPEED, 0.0
                    last_key_time = time.time()
                elif ch == 's':
                    jog_linear, jog_angular = -JOG_LINEAR_SPEED, 0.0
                    last_key_time = time.time()
                elif ch == 'a':
                    jog_linear, jog_angular = 0.0, JOG_ANGULAR_SPEED
                    last_key_time = time.time()
                elif ch == 'd':
                    jog_linear, jog_angular = 0.0, -JOG_ANGULAR_SPEED
                    last_key_time = time.time()
                elif ch == ' ':
                    jog_linear, jog_angular = 0.0, 0.0
                    last_key_time = 0.0

            if time.time() - last_key_time > JOG_KEY_TIMEOUT_S:
                jog_linear, jog_angular = 0.0, 0.0

            base.move(linear=jog_linear, angular=jog_angular)

            if reference_pose is not None and time.time() - last_status > 0.5:
                pose = position.current_pose()
                if pose is not None:
                    dist = math.hypot(pose[0] - reference_pose[0], pose[1] - reference_pose[1])
                    sys.stdout.write(f'\r  {dist:.3f}m from HOME   ')
                    sys.stdout.flush()
                last_status = time.time()
    finally:
        base.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print()


def run_blocking(node, fn, *fn_args, **fn_kwargs):
    """Runs a blocking call (go_to_pose loops on time.sleep reading odom) in a
    background thread while spinning node on this thread, so odom/IMU keep updating."""
    box = {}

    def target():
        box['value'] = fn(*fn_args, **fn_kwargs)

    t = threading.Thread(target=target, daemon=True)
    t.start()
    while t.is_alive():
        rclpy.spin_once(node, timeout_sec=0.05)
    t.join()
    return box.get('value')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--material', required=True, help='label to save/test this bin offset under')
    ns, _ = parser.parse_known_args(sys.argv[1:])

    if ns.record == ns.test:
        print('Pass exactly one of --record or --test')
        sys.exit(1)

    rclpy.init(args=args)
    node = Node('calibrate_home_bin')
    log = node.get_logger()

    config = load_config()
    position = PositionTracker(node, config, log)
    base = RobotBase(node, config, log)
    imu = ImuSensor(node, config, log)
    calib_path = resolve_path(config, config.get('bins', {}).get('calibration_file', 'config/bin_position_calibration.json'))

    if ns.record:
        jog_until_enter(node, base, position, '\nJog the robot to HOME, then press ENTER to capture it...')
        home = position.current_pose()
        if home is None:
            print('No /odom data yet - aborting.')
            sys.exit(1)
        home_x, home_y, home_yaw = home
        print(f'HOME captured: x={home_x:.3f} y={home_y:.3f} yaw={home_yaw:.1f}deg')

        jog_until_enter(
            node, base, position,
            f'\nJog the robot to the "{ns.material}" bin, then press ENTER to save it...',
            reference_pose=home,
        )
        bin_pose = position.current_pose()
        bin_x, bin_y, bin_yaw = bin_pose

        offset = {
            'dx': round(bin_x - home_x, 4),
            'dy': round(bin_y - home_y, 4),
            'dyaw_deg': round(normalize_deg(bin_yaw - home_yaw), 2),
        }
        print(f'Offset from HOME: {offset}')

        data = {}
        if os.path.isfile(calib_path):
            with open(calib_path) as f:
                data = json.load(f)
        data[ns.material] = offset
        os.makedirs(os.path.dirname(calib_path) or '.', exist_ok=True)
        with open(calib_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f'Saved to {calib_path}')
        print('\nPaste into config.yaml under bins.targets:')
        print(f"  - material: {ns.material}\n    offset: {{dx: {offset['dx']}, dy: {offset['dy']}, dyaw_deg: {offset['dyaw_deg']}}}")

    else:
        if not os.path.isfile(calib_path):
            print(f'No calibration file at {calib_path} - run --record first.')
            sys.exit(1)
        with open(calib_path) as f:
            data = json.load(f)
        if ns.material not in data:
            print(f'No offset recorded for "{ns.material}" in {calib_path}.')
            sys.exit(1)
        offset = data[ns.material]

        jog_until_enter(node, base, position, '\nJog the robot to HOME, then press ENTER to start the test drive...')
        home = position.current_pose()
        if home is None:
            print('No /odom data yet - aborting.')
            sys.exit(1)
        home_x, home_y, home_yaw = home

        target = resolve_offset_target(home_x, home_y, home_yaw, offset)
        print(f'Driving to {ns.material} bin at {target}...')
        success = run_blocking(node, position.go_to_pose, base, imu, *target, running_flag=lambda: True)
        print('Arrived.' if success else 'Failed to reach target (see nav log above).')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
