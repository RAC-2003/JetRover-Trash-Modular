"""Shared helpers for the test_* nodes - not an entry point itself."""

import threading
import time

import rclpy


def spin_until(node, condition_fn, timeout_s):
    """Spins node until condition_fn() is True or timeout_s elapses.
    Returns True if the condition was met."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition_fn():
            return True
    return False


def spin_for(node, duration_s):
    """Spins node for duration_s so its timers (e.g. RobotBase's watchdog
    publish loop) actually fire."""
    deadline = time.time() + duration_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def run_blocking(node, fn, *args, **kwargs):
    """Runs a blocking call (e.g. Movement.rotate_by_angle, which loops on
    time.sleep reading sensor state) in a background thread while spinning
    node on the caller's thread, so subscriptions/timers keep updating.
    Mirrors how the pipeline node runs its state machine: spin on the main
    thread, blocking hardware calls on a worker thread."""
    box = {}

    def target():
        box['value'] = fn(*args, **kwargs)

    t = threading.Thread(target=target, daemon=True)
    t.start()
    while t.is_alive():
        rclpy.spin_once(node, timeout_sec=0.05)
    t.join()
    return box.get('value')


def banner(logger, text):
    logger.info('=' * 60)
    logger.info(text)
    logger.info('=' * 60)


def result(logger, passed, summary):
    banner(logger, f'RESULT: {"PASS" if passed else "FAIL"} - {summary}')
    return passed
