"""Generic command watchdog: a command is zeroed out if nobody refreshes it
within `timeout_s`. Used by hardware.base.RobotBase so the robot can never
keep moving because a thread stalled - the old project had no such guarantee
on trash_sorter's own cmd_vel commands."""

import time


class CommandWatchdog:
    def __init__(self, timeout_s):
        self.timeout_s = timeout_s
        self._last_refresh = 0.0

    def refresh(self):
        self._last_refresh = time.time()

    def expired(self):
        return (time.time() - self._last_refresh) > self.timeout_s
