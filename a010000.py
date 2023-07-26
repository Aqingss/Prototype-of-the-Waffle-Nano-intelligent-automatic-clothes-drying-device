"""Micropython module for stepper motor driven by Easy Driver."""
from machine import Pin
from utime import sleep_us


class A010000:
    """Class for stepper motor driven by Easy Driver."""

    def __init__(self):
        p11= Pin(11,Pin.OUT)
        p12= Pin(12,Pin.OUT)
        p13= Pin(13,Pin.OUT)
        p14= Pin(14,Pin.OUT)
        """Initialise stepper."""
        self.stp = p12
        self.dir = p13
        self.slp = p11
        self.en  = p14

        self.step_time = 20  # us
        self.steps_per_rev = 400
        self.current_position = 0
        
    def enable(self):
        """Power on stepper."""
        self.en.value(1)
        
    def disable(self):
        """Power on stepper."""
        self.en.value(0)
        
    def power_on(self):
        """Power on stepper."""
        self.slp.value(1)

    def power_off(self):
        """Power off stepper."""
        self.slp.value(0)
        self.current_position = 0

    def steps(self, step_count):
        """Rotate stepper for given steps."""
        self.dir.value(0 if step_count > 0 else 1)
        for i in range(abs(step_count)):
            self.stp.value(1)
            sleep_us(self.step_time)
            self.stp.value(0)
            sleep_us(self.step_time)
        self.current_position += step_count

    def rel_angle(self, angle):
        """Rotate stepper for given relative angle."""
        angle=angle/1.0
        steps = int(angle / 360 * self.steps_per_rev)
        self.steps(steps)

    def abs_angle(self, angle):
        """Rotate stepper for given absolute angle since last power on."""
        angle=angle/1.0
        steps = int(angle / 360 * self.steps_per_rev)
        steps -= self.current_position % self.steps_per_rev
        self.steps(steps)

    def revolution(self, rev_count):
        """Perform given number of full revolutions."""
        self.steps(rev_count * self.steps_per_rev)

    def set_step_time(self, us):
        """Set time in microseconds between each step."""
        if us < 20:  # 20 us is the shortest possible for esp8266
            self.step_time = 20
        else:
            self.step_time = us