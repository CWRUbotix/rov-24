#  Control a 5V PWM fan speed with the lgpio library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import lgpio
import time

# Configuration
FAN = 12  # pin used to drive PWM fan
FREQ = 50

h = lgpio.gpiochip_open(0)

try:
    while True:
        # Turn the fan off
        lgpio.tx_pwm(h, FAN, FREQ, 0)
        time.sleep(1)

        # # Turn the fan to medium speed
        # lgpio.tx_pwm(h, FAN, FREQ, 50)
        # time.sleep(10)

        print("on")
        # Turn the fan to max speed
        lgpio.tx_pwm(h, FAN, FREQ, 100)
        time.sleep(10)

except KeyboardInterrupt:
    # Turn the fan to medium speed
    lgpio.tx_pwm(h, FAN, FREQ, 50)
    lgpio.gpiochip_close(h)
