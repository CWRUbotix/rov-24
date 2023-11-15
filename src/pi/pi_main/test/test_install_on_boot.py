import os

from pi_main.install_on_boot import main

EXPECTED_SYSTEM_FILES = ["/etc/ros/humble/cwrubotix_pi.d/.installed_files",
                         "/etc/ros/humble/cwrubotix_pi.d/pi.launch.py",
                         "/etc/systemd/system/multi-user.target.wants/cwrubotix_pi.service",
                         "/lib/systemd/system/cwrubotix_pi.service",
                         "/usr/sbin/cwrubotix_pi-start",
                         "/usr/sbin/cwrubotix_pi-stop"]


def test_install_on_boot():
    """Test that file copying and systemd are made."""
    main()

    actual_rules_files = set(os.listdir(os.path.join("/etc", "udev", "rules.d")))
    expected_rules_files = set(["i2c.rules", "camera.rules", "pixhawk.rules"])
    assert expected_rules_files.issubset(actual_rules_files)

    for file in EXPECTED_SYSTEM_FILES:
        assert os.path.isfile(file)
