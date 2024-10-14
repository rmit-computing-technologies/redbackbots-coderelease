import logging
import os
import inspect

LOG_COLOURS = {
    'RESET': '\033[0m',    # Reset
    'DEBUG': '\033[0m',    # Reset
    'INFO': '\033[96m',    # Cyan
    'WARNING': '\033[93m', # Yellow
    'ERROR': '\033[91m',   # Red
    'CRITICAL': '\033[91;1m' # Bold Red
}

logger = logging.getLogger('behaviour')
logger.setLevel(logging.DEBUG)  # Set this to logging.INFO for comp

log_path = os.getenv('LOG_PATH', '/var/volatile/redbackbots')
log_folders = os.listdir(log_path)
log_folders.sort()
log_folder = '%s/%s' % (log_path, log_folders[-1])
while not os.path.isdir(log_folder):
    log_folders.pop()
    log_folder = '%s/%s' % (log_path, log_folders[-1])

file_handler = logging.FileHandler(log_folder+"/behaviour")
logger.addHandler(file_handler)

# Comment this handler out for competition
console_handler = logging.StreamHandler()
logger.addHandler(console_handler)

def log_with_colour(level, msg, *args, **kwargs):
    frame = inspect.currentframe().f_back.f_back
    filename = os.path.basename(inspect.getframeinfo(frame).filename)
    line_number = frame.f_lineno

    display_level = f"[{level}]"
    message = f" {display_level:10} {filename}:{line_number:<3} - {msg}"
    coloured_msg = f"{LOG_COLOURS[level]}{message}{LOG_COLOURS['RESET']}"
    getattr(logger, level.lower())(coloured_msg, *args, **kwargs)

# Not sure how useful it would be to timestamp *every* message,
# might leave this to the user when required
# formatter = logging.Formatter(fmt = '%(created)f:%(message)s')

def debug(msg, *args, **kwargs):
    log_with_colour('DEBUG', msg, *args, **kwargs)

def info(msg, *args, **kwargs):
    log_with_colour('INFO', msg, *args, **kwargs)

def warning(msg, *args, **kwargs):
    log_with_colour('WARNING', msg, *args, **kwargs)

def error(msg, *args, **kwargs):
    log_with_colour('ERROR', msg, *args, **kwargs)

def critical(msg, *args, **kwargs):
    log_with_colour('CRITICAL', msg, *args, **kwargs)