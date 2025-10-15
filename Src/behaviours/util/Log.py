import logging
import os
import inspect
import configparser
import time
from util.say import say_payload

LOG_COLOURS = {
    'RESET': '\033[0m',         # Reset
    'DEBUG': '\033[0m',         # Reset
    'INFO': '\033[96m',         # Cyan
    'WARNING': '\033[93m',      # Yellow
    'ERROR': '\033[91m',        # Red
    'CRITICAL': '\033[91;1m',   # Bold Red
    'SAY': "\033[1;35m"         # Magenta
}

CONFIG_PATH = os.getenv("CONFIG_PATH", "/home/nao/config/redbackbots.cfg")
LOG_PATH = os.getenv("LOG_PATH", "/var/volatile/redbackbots")

logger = None
use_say = False

def init_logger():
    global logger
    logger = logging.getLogger('behaviour')
    logger.setLevel(logging.DEBUG)  # Set this to logging.INFO for comp

    log_folders = os.listdir(LOG_PATH)
    log_folders.sort()
    log_folder = '%s/%s' % (LOG_PATH, log_folders[-1])
    while not os.path.isdir(log_folder):
        log_folders.pop()
        log_folder = '%s/%s' % (LOG_PATH, log_folders[-1])

    file_handler = logging.FileHandler(log_folder + "/behaviour")
    logger.addHandler(file_handler)

    logging_config = get_logging_config()
    if logging_config['log.say']:
        # Change debug/log.say in redbackbots.cfg to FALSE for competitions (To avoid unneeded espeak usage)
        global use_say
        use_say = True
    if logging_config['log.stdout']:
        # Change debug/log.stdout in redbackbots.cfg to FALSE for competitions (To avoid unneeded console logs)
        console_handler = logging.StreamHandler()
        logger.addHandler(console_handler)

def get_logging_config():
    config = configparser.ConfigParser()
    config.read(CONFIG_PATH)
    behaviour_section = config['debug']
    return {
        "log.stdout": behaviour_section.getboolean("log.stdout", fallback=True),
        "log.say": behaviour_section.getboolean("log.say", fallback=True)
    }

def log_with_colour(level, msg, say, *args, **kwargs):
    if not logger:
        print("ERROR: logger is not initialised")
    if say and use_say:
        say_payload({
            "message": msg,
            "level": level
        })

    frame = inspect.currentframe().f_back.f_back
    filename = os.path.basename(inspect.getframeinfo(frame).filename)
    line_number = frame.f_lineno

    display_level = f"[{level}]"
    message = f" {display_level:10} {filename}:{line_number:<3} - {msg}"
    colour = LOG_COLOURS["SAY"] if say else LOG_COLOURS[level]
    coloured_msg = f"{colour}{message}{LOG_COLOURS['RESET']}"
    getattr(logger, level.lower())(coloured_msg, *args, **kwargs)

# Not sure how useful it would be to timestamp *every* message,
# might leave this to the user when required
# formatter = logging.Formatter(fmt = '%(created)f:%(message)s')

def debug(msg, say=False, *args, **kwargs):
    log_with_colour('DEBUG', msg, say, *args, **kwargs)

def info(msg, say=False, *args, **kwargs):
    log_with_colour('INFO', msg, say, *args, **kwargs)

def warning(msg, say=False, *args, **kwargs):
    log_with_colour('WARNING', msg, say, *args, **kwargs)

def error(msg, say=False, *args, **kwargs):
    log_with_colour('ERROR', msg, say, *args, **kwargs)

def critical(msg, say=False, *args, **kwargs):
    log_with_colour('CRITICAL', msg, say, *args, **kwargs)

def sound(sound_file):
    """
    Play a sound file using aplay.
    :param sound_file: Path to the sound file to play.
    :param say: If True, will also use say_payload to announce the sound.
    """

    if use_say:
        log_with_colour('INFO', f"Playing sound: {sound_file}", say=False)
        say_payload({
            "message": sound_file,
            "level": 'SOUND'
        })
    else:
        log_with_colour('INFO', f"Would have played sound: {sound_file}", say=False)

if __name__ == "__main__":
    init_logger()
    debug("This is a debug message")
    info("This is an info message")
    warning("This is a warning message")
    error("This is an error message")
    critical("This is a critical message")

    debug("This is a debug message, with say", say=True)
    info("This is an info message, with say", say=True)
    warning("This is a warning message, with say", say=True)
    error("This is an error message, with say", say=True)
    critical("This is a critical message, with say", say=True)
    critical("This is a critical message, with say", say=True)
    critical("This is a critical message, with say", say=True)


    time.sleep(20)
