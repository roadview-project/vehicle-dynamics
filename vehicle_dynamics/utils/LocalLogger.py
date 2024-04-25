import logging
import sys


class LocalLogger(logging.Logger):
    """Logger util for the package"""
    loggers = set()

    def __init__(self, name, format="%(asctime)s | %(levelname)s | %(funcName)s %(lineno)d | %(message)s", level=logging.INFO):
        # Initial construct.
        self.format = format
        self.level = level
        self.name = name

        # Logger configuration.
        self.console_formatter = logging.Formatter(self.format)
        self.console_logger = logging.StreamHandler(sys.stdout)
        self.console_logger.setFormatter(self.console_formatter)

        # Complete logging config.
        self.logger = logging.getLogger(name)
        if name not in self.loggers:
            self.loggers.add(name)
            self.logger.setLevel(self.level)
            self.logger.addHandler(self.console_logger)
