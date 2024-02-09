import lgpio


def main():
    while True:
        # Pins used for GPIO
        detect1, detect2, detect3 = 2, 4, 6
        # GPIO Boilerplate
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(h, detect1)
        lgpio.gpio_claim_input(h, detect2)
        lgpio.gpio_claim_input(h, detect3)

        # Read Data
        data1 = lgpio.gpio_read(h, detect1)
        data2 = lgpio.gpio_read(h, detect2)
        data3 = lgpio.gpio_read(h, detect3)

        print("Pin 2: %s" % data1)
        print("\n Pin 4: %s" % data2)
        print("\n Pin 6: %s" % data3)

        if data1 or data2 or data3:
            print("\n \n Bad Flooding Thing")


if __name__ == '__main__':
    main()
