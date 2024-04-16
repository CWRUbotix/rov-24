import lgpio

# Pins used for GPIO
DETECT_PIN = 17


def main() -> None:
    # GPIO Boilerplate
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(h, DETECT_PIN)

    data: int = lgpio.gpio_read(h, DETECT_PIN)

    first_run = True

    while True:
        # Read Data
        old_data: int = data

        data = lgpio.gpio_read(h, DETECT_PIN)

        if data != old_data or first_run:
            print("Pin 17: %s" % data)

            if data:
                print("\nBad Flooding Thing")

            first_run = False


if __name__ == '__main__':
    main()
