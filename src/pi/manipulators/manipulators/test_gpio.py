#  Control a 5V PWM fan speed with the lgpio library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import lgpio
import time

# Configuration
SERVO_PIN = 12  # pin used to drive PWM fan
FREQ = 50

gpio_handle = lgpio.gpiochip_open(0)


def test_gpio(width: int, freq: int) -> None:
    lgpio.tx_servo(gpio_handle, SERVO_PIN, width, freq)


try:
    print('Starting loop')
    while True:
        # Turn the fan off
        test_gpio(2000, 50)
        time.sleep(2)

        print('on')
        # Turn the fan to medium speed
        test_gpio(1000, 200)
        time.sleep(2)

        print('more on')
        # Turn the fan to max speed
        test_gpio(500, 400)
        time.sleep(2)

except KeyboardInterrupt:
    # Turn the fan to medium speed
    lgpio.tx_pwm(gpio_handle, SERVO_PIN, FREQ, 50)
    lgpio.gpiochip_close(gpio_handle)
