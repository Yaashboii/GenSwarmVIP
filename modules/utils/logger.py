import logging
from enum import Enum

__all__ = ['setup_logger', 'LoggerLevel', 'format_log_message']

# ANSI 转义码
_ANSI_COLOR_CODES = {
    'RESET': '\033[0m',
    'BOLD': '\033[1m',
    'UNDERLINE': '\033[4m',
    'BLACK': '\033[30m',
    'RED': '\033[31m',
    'GREEN': '\033[32m',
    'YELLOW': '\033[33m',
    'BLUE': '\033[34m',
    'MAGENTA': '\033[35m',
    'CYAN': '\033[36m',
    'WHITE': '\033[37m',
}


class _ColoredFormatter(logging.Formatter):
    def format(self, record):
        # Apply color based on log level
        log_level_color = {
            logging.DEBUG: _ANSI_COLOR_CODES['BLUE'],
            logging.INFO: _ANSI_COLOR_CODES['GREEN'],
            logging.WARNING: _ANSI_COLOR_CODES['YELLOW'],
            logging.ERROR: _ANSI_COLOR_CODES['RED'],
            logging.CRITICAL: _ANSI_COLOR_CODES['MAGENTA']
        }.get(record.levelno, _ANSI_COLOR_CODES['RESET'])

        # Customize log record message with color
        record.msg = f"{log_level_color}{record.msg}{_ANSI_COLOR_CODES['RESET']}"

        # Apply color to other parts of the log record
        record.levelname = f"{log_level_color}[{record.levelname}]{_ANSI_COLOR_CODES['RESET']}"
        record.name = f"{log_level_color}[{record.name}]{_ANSI_COLOR_CODES['RESET']}"
        if isinstance(record.created, float):
            record.created = f"{log_level_color}[{float(record.created):.6f}]{_ANSI_COLOR_CODES['RESET']}"

        return super(_ColoredFormatter, self).format(record)


class LoggerLevel(Enum):
    INFO = logging.INFO
    DEBUG = logging.DEBUG
    CRITICAL = logging.CRITICAL
    ERROR = logging.ERROR
    WARNING = logging.WARNING


def setup_logger(name, level=LoggerLevel.INFO):
    """
    Set up a logger and return it.

    Args:
        name (str): The logger name.
        level (str): The logging level as a string ('DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL').

    Returns:
        logging.Logger: The configured logger.
    """
    # Convert the level string to the corresponding integer value
    # level = getattr(logging, level.upper(), logging.INFO)
    logger = logging.getLogger(name)
    logger.setLevel(level.value)
    # Create a console handler
    ch = logging.StreamHandler()
    # Use the custom colored formatter
    formatter = _ColoredFormatter('%(created)s%(name)s%(levelname)s %(message)s')
    ch.setFormatter(formatter)
    # Add the console handler to the logger
    logger.addHandler(ch)

    return logger


def format_log_message(label: str, message: str) -> str:
    """Format a log message."""
    separator = '=' * 20
    return f"{label}:\n{separator}\n{message}\n{separator}"


if __name__ == '__main__':
    # 设置全局logger，日志级别为DEBUG
    logger = setup_logger("GlobalLogger", LoggerLevel.DEBUG)
    # 例子：记录一个DEBUG消息
    logger.debug("这是一个DEBUG消息")
    # 例子：记录一个INFO消息
    logger.info("这是一个INFO消息")
    # 例子：记录一个WARNING消息
    logger.warning("这是一个WARNING消息")
    # 例子：记录一个ERROR消息
    logger.error("这是一个ERROR消息")
    # 例子：记录一个CRITICAL消息
    logger.critical("这是一个CRITICAL消息")
