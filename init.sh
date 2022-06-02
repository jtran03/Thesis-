# Initialisatin Script 

#### installing ROS ####
echo "##########################################################"
echo "Installing ROS-melodic"
echo "##########################################################"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git
rm -rf ~/catkin_ws/src 
mkdir -p ~/catkin_ws/src 
echo "Creating Catkin Workspace" 
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

### Installing nodejs ###
mkdir -p ~/Desktop/VirtualEnv
cd ~/Desktop/VirtualEnv
sudo apt install python3-venv -y
python3 -m venv my-python-env
source ~/Desktop/VirtualEnv/my-python-env/bin/activate
pip install nodeenv
nodeenv --node=16.15.1 --npm=8.11.0 ~/Desktop/VirtualEnv/env
. env/bin/activate
deactivate_node
deactivate

# Installing ZED 
echo "##########################################################"
echo "Installing ZED SDK"
echo "##########################################################"
cd ~/Desktop/
mkdir -p tempfiles
cd tempfiles/
echo "##########################################################"
echo "################### Downloading ZED SDK ##################"
echo "##########################################################"
wget  -O zedsdk.run "https://download.stereolabs.com/zedsdk/3.7/l4t32.7/jetsons"
chmod +x zedsdk.run
./zedsdk.run -- silent

echo "Installing ROS Packages"
cd ~/catkin_ws/src
# ZED 2 
git clone https://github.com/stereolabs/zed-ros-wrapper.git
git clone https://github.com/stereolabs/zed-ros-interfaces.git

# Arduino Related Packages
sudo apt-get install ros-melodic-rosserial-arduino -y

# ZLAC8015D Packages
cp -R ~/Desktop/Thesis-/ROS\ Packages/zlac8015d ~/catkin_ws/src 

$ IMU Brick 2.0 
git clone https://github.com/SteveMacenski/tinkerforge_imu_ros.git
rm -f ~/catkin_ws/tinkerforge_imu_ros/src/brick_imu_v2.cpp
cp ~/Desktop/Thesis-/Drivers/imubrick2.0/brick_imu_v2.cpp ~/catkin_ws/tinkerforge_imu_ros/src/

# RPLidar 


# Web Server 
sudo apt-get install ros-melodic-rosbridge-server -y

#Installing 
echo "Compiling"
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash
rm -rf ~/Desktop/tempfiles
cd ~/Desktop
