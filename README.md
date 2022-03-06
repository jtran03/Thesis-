# PIZZA DELIVERY SIMULATOR in DOCKER

# Usage
1. Run `python runContainer.py` (or python3)
    - This will build and start the container.
    - This DOES NOT push and build the code, you will need to run `python updateCode.py` in a separate shell - keep it open so you can quickly push changes.
2. Using [VNCViewer](https://www.realvnc.com/en/connect/download/viewer/), connect to `localhost:5900`.
3. The first time or to update PizzaCommander, you can simply run `python updateCode.py`.
    - This will docker copy into the running container and then catkin_make for you.
    - If VNC crashes, go to the container that is running runContainer and type in `/openvnc.sh` to restart vnc.
4. Run `python doLaunch.py end2end_random_house.launch` to run the code
IF you have `rviz_with_tfprefix.py file not found` error, then:
- In the bottom right corner of your IDE, there will be a CRLF button
- Click it, click 'LF', press save, run `python updateCode.py` again, then do step 4 again
- If that still doesnt work, open a shell in vnc and type `chmod +x /root/catkin_ws/src/pizza_commander/scripts/rviz_with_tfprefix.py` and run step 4 again

# Running our code on a non-docker machine
- Move the `pizza_commander` directory into your catkin ws src folder.
- Run `catkin_make`.
- Run `roslaunch end2end_random_house.launch`

# TODO
See https://docs.google.com/spreadsheets/d/1tIskMsOUkLEIlw_gb4BA-x9y2Pl94T1kxwfUxwTYyNQ/edit#gid=0

/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch#


rviz -d /root/catkin_ws/src/pizza_commander/launch/navigation_intermediaries/navigation_all.rviz
