from time import sleep

import tsys01  # https://github.com/bluerobotics/tsys01-python


def debug_log() -> None:
    sensor = tsys01.TSYS01()  # Use default I2C bus 1

    sensor.init()

    while True:
        sensor.read()
        print(
            sensor.temperature(),  # Get temperature in default units (Centigrade)
            '\t',
            sensor.temperature(tsys01.UNITS_Farenheit)
        )
        sleep(1)


if __name__ == "__main__":
    debug_log()
