"""Thin helpers so every module logs in the same [INFO]/[ERROR]-friendly shape
that ros2's console formatter already produces. Kept intentionally tiny."""


def log_info(logger, msg):
    logger.info(msg)


def log_warn(logger, msg):
    logger.warn(msg)


def log_error(logger, msg):
    logger.error(msg)


def log_step(logger, step, msg):
    """Used by test_nodes to print the TEST START -> ... -> PASS/FAIL shape."""
    logger.info(f'[{step}] {msg}')
