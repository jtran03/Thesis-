import os
import time
import argparse
# Build

parser = argparse.ArgumentParser()
parser.add_argument("--quick", action="store_true")
parser.add_argument("--nobuild", action="store_true")

args = parser.parse_args()


os.system("docker cp shellCommander pizzabotsim:/root/catkin_ws/src")

if not args.nobuild:
    if args.quick:
        os.system("docker exec -it pizzabotsim /bin/bash -c \"source /opt/ros/melodic/setup.bash && cd /root/catkin_ws && catkin_make --only-pkg-with-deps pizza_commander\"")
    else:
        os.system("docker exec -it pizzabotsim /bin/bash -c \"source /opt/ros/melodic/setup.bash && cd /root/catkin_ws && catkin_make \"")

os.system("docker exec -it turtlebot /bin/bash -c \"chmod +x /root/catkin_ws/src/shellCommander/scripts/rviz_with_tfprefix.py \"")
