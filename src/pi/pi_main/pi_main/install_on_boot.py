import os
import subprocess
import pathlib

from ament_index_python.packages import get_package_share_directory
from robot_upstart.job import Job


def main():
    pi_main_share = get_package_share_directory('pi_main')

    launch_dir = os.path.join(pi_main_share, 'launch')
    launch_src = os.path.join(launch_dir, 'pi_launch.py')
    launch_dst = os.path.join(launch_dir, 'pi.launch.py')

    try:
        os.unlink(launch_dst)
    except FileNotFoundError:
        pass

    os.symlink(launch_src, launch_dst)

    file_location = pathlib.Path(__file__).parent.resolve()
    udev_script = os.path.join(file_location, 'udev_copy.py')

    try:
        p = subprocess.run(['sudo'] + ['python'] + [udev_script] + [pi_main_share],
                           capture_output=True, check=True)
    except subprocess.CalledProcessError as e:
        print(e)
        assert False

    print(p.stdout.decode())

    cwrubotix_job = Job(name='cwrubotix_pi', rmw='rmw_cyclonedds_cpp')
    cwrubotix_job.symlink = True
    cwrubotix_job.uninstall()
    cwrubotix_job.add(package='pi_main', filename='launch/pi.launch.py')
    cwrubotix_job.install()
