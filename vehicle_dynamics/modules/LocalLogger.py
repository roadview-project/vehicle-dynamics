"""
Vehicle Dynamic Model - Logger Class

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
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
