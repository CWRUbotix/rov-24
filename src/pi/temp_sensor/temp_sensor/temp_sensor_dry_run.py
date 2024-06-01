import tsys01
# https://github.com/bluerobotics/tsys01-python


def debug_log() -> None:
    sensor = tsys01.TSYS01()  # Use default I2C bus 1

    sensor.init()

    sensor.read()

    sensor.temperature()  # Get temperature in default units (Centigrade)


if __name__ == "__main__":
    debug_log()
