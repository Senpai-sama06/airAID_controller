# controller/util.py
import logging
import sys

def setup_logger(log_level_str):
    """Configures and returns a standard logger."""
    # Convert the string log level to a logging constant
    log_level = getattr(logging, log_level_str.upper(), logging.INFO)

    # Get the logger for our package
    logger = logging.getLogger("AIRAID_CONTROLLER")
    logger.setLevel(log_level)

    # This prevents log messages from being duplicated
    logger.propagate = False

    # Add the handler only if it doesn't have one already
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        # Define the format for our log messages
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger