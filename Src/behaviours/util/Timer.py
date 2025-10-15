"""
A timing utility that allows you to measure time and meet time targets.
`blackboard.vision.timestamp` is used to measure time,
so the time units are microseconds.

The restart(), start() and stop() methods are chainable, e.g.:

countdown_timer = Timer(1000000).start()
countdown_timer.restart().start()
1000000 = 1 second
"""

blackboard = None

def update_timer(newBlackboard):
    """
    Updates the Timer.py global variables, i.e. the blackboard.

    Callable via `Timer.update_timer(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard


class Timer():
    """A simple timer to measure time"""
    def __init__(self, time_target=0, target_seconds=None):
        """
        :param time_target: The target time in microseconds.
        :param target_seconds: The target time in seconds. If this is set, it will
                              override the time_target parameter."""
        if target_seconds is not None:
            self.time_target = self.s_to_us(target_seconds)
        else:
            self.time_target = time_target
        self.running = False
        self.elapsed_time = 0
        self.start_time = blackboard.vision.timestamp
        self.restart()

    def running_finished(self):
        """
        Gets if finished and running.
        Returns:
            bool: True if the timer is both running and finished, False otherwise.
        """
        return self.running and self.finished()

    def restart(self):
        """Reset the timer. If it's running, it will keep running."""
        self.elapsed_time = 0
        self.start_time = blackboard.vision.timestamp
        return self

    def start(self):
        """Start the timer.  Does nothing if it's already running"""
        if not self.running:
            self.start_time = blackboard.vision.timestamp
            self.running = True
        return self

    def stop(self):
        """Stops the timer. Does nothing if it's already stopped."""
        if self.running:
            self.elapsed_time += blackboard.vision.timestamp - self.start_time
            self.running = False
        return self

    def elapsed(self):
        """Returns how much time has elapsed so far"""
        return self.elapsed_time + blackboard.vision.timestamp - self.start_time

    def elapsed_seconds(self):
        """Returns how much time has elapsed so far in seconds"""
        return self.elapsed() / 1_000_000.0

    def finished(self):
        """Returns whether the timer has reached its target time"""
        return self.elapsed() >= self.time_target

    @staticmethod
    def ms_to_us(ms):
        """
        Utility function to convert milliseconds to microseconds.
        
        :param ms: Milliseconds to convert.
        :return: Microseconds to parse to timer.
        """
        return ms * 1_000

    @staticmethod
    def s_to_us(s):
        """
        Utility function to convert seconds to microseconds.
        
        :param s: Seconds to convert.
        :return: Microseconds to parse to timer.
        """
        return s * 1_000_000


class WallTimer(object):
    def __init__(self, timeTarget=0):
        self.timeTarget = timeTarget
        self.running = False
        self.startTime = blackboard.vision.timestamp

    def restart(self):
        self.startTime = blackboard.vision.timestamp
        return self

    def elapsed(self):
        return blackboard.vision.timestamp - self.startTime

    def elapsedSeconds(self):
        return self.elapsed() / 1_000_000.0

    def finished(self):
        return blackboard.vision.timestamp - self.startTime >= self.timeTarget


    @staticmethod
    def ms_to_us(ms):
        """
        Utility function to convert milliseconds to microseconds.
        
        :param ms: Milliseconds to convert.
        :return: Microseconds to parse to timer.
        """
        return ms * 1_000


    @staticmethod
    def s_to_us(s):
        """
        Utility function to convert seconds to microseconds.
        
        :param s: Seconds to convert.
        :return: Microseconds to parse to timer.
        """
        return s * 1_000_000

    

class BumperTimer(object):
    """
    For exclusive use in ObstacleAvoidance.py
    Used to record the amount of times a bumper hits and object
    """
    def __init__(self, timeTarget=0):
        self.timeTarget = timeTarget
        self.startTime = blackboard.vision.timestamp
        self.bumperActivations = 0

    def restart(self):
        self.startTime = blackboard.vision.timestamp
        return self

    def elapsed(self):
        return blackboard.vision.timestamp - self.startTime

    def elapsedSeconds(self):
        return self.elapsed() / 1000000.0
    
    def add_activation(self):
        self.bumperActivations += 1

    def reset_activations(self):
        self.bumperActivations = 0

    def get_num_activations(self):
        return self.bumperActivations
    
    def finished(self):
        return blackboard.vision.timestamp - self.startTime >= self.timeTarget
