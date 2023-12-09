import time

from smbus2 import SMBus

TCA9555_ADDRESS = 0x20


def main() -> None:
    """
    Test relay board by blinking lights.
    """
    with SMBus() as bus:
        while True:

            bus.write_byte(TCA9555_ADDRESS, 0b00000000)
            time.sleep(1000)
            bus.write_byte(TCA9555_ADDRESS, 0b11111111)


if __name__ == '__main__':
    main()
