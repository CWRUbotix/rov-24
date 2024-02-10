import lgpio


def main() -> None:
    # Pins used for GPIO
    detect1, detect2, detect3 = 2, 4, 6
    # GPIO Boilerplate
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(h, detect1)
    lgpio.gpio_claim_input(h, detect2)
    lgpio.gpio_claim_input(h, detect3)

    data1 = lgpio.gpio_read(h, detect1)
    data2 = lgpio.gpio_read(h, detect2)
    data3 = lgpio.gpio_read(h, detect3)

    firstRun = True

    while True:
        # Read Data
        olddata1: int = data1
        olddata2: int = data2
        olddata3: int = data3

        data1 = lgpio.gpio_read(h, detect1)
        data2 = lgpio.gpio_read(h, detect2)
        data3 = lgpio.gpio_read(h, detect3)

        if data1 != olddata1 or data2 != olddata2 or data3 != olddata3 or firstRun:
            print("Pin 2: %s" % data1)
            print("Pin 4: %s" % data2)
            print("Pin 6: %s" % data3)

            if data1 or data2 or data3:
                print("\n Bad Flooding Thing")
            
            firstRun = False


if __name__ == '__main__':
    main()
