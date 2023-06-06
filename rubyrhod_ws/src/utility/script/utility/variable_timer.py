from rospy import Rate, Timer, Duration
from rospy.timer import TimerEvent
import rospy
import time

class VariableRate(Rate):
    """Convenience class derived from the rate class
        Expands on it's capabilities by allowing the change in frequency
    """

    def set_frequency(self, hz:float, reset:bool=False):
        """set the desired frequency

        :param hz: desired frequency in hz
        :type hz: float
        :param reset: if True, timer is reset when rostime moved backward. [default: False]
        :type reset: bool
        """
        self.sleep_dur = rospy.rostime.Duration(0, int(1e9/hz))
        self._reset = reset

class VariableTimer(Timer):
    """Convenience class for calling a callback at a specified rate
        Derived from the Timer class, expands on it's capabilities by
        allowing changing the period between callbacks during runtime
    """

    def set_period(self, period:Duration, reset:bool=False):
        """set desired period between callbacks

        :param period: _description_
        :type period: Duration
        :param reset: _description_, defaults to False
        :type reset: bool, optional
        """

        self._period = period
        self._reset = reset
        self.r.set_frequency(1.0 / self._period.to_sec(), reset=self._reset)

    def run(self):
        self.r = VariableRate(1.0 / self._period.to_sec(), reset=self._reset)
        current_expected = rospy.rostime.get_rostime() + self._period
        last_expected, last_real, last_duration = None, None, None
        while not rospy.core.is_shutdown() and not self._shutdown:
            try:
                self.r.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                if rospy.core.is_shutdown():
                    break
                raise
            if self._shutdown:
                break
            current_real = rospy.rostime.get_rostime()
            start = time.time()
            self._callback(TimerEvent(last_expected, last_real, current_expected, current_real, last_duration))
            if self._oneshot:
                break
            last_duration = time.time() - start
            last_expected, last_real = current_expected, current_real
            current_expected += self._period