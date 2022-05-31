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
rm -rf ~/catkin_ws
mkdir -p ~/catkin_ws/src 
echo "Creating Catkin Workspace" 
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc


# Installing ZED 
echo "##########################################################"
echo "Installing ZED SDK"
echo "##########################################################"
cd ~/Desktop/
mkdir -p tempfiles
cd tempfiles/

{
if [ ! -f ~/usr/local/zed/get_python_api.py ]; then
  echo "##########################################################"
  echo "################### Downloading ZED SDK ##################"
  echo "##########################################################"
  wget  -O zedsdk.run "https://download.stereolabs.com/zedsdk/3.7/l4t32.7/jetsons"
  chmod +x zedsdk.run
  ./zedsdk.run -- silent
fi
}
rm -rf tempfiles


echo "Installing ZED ROS Packages"
cd ~/catkin_ws/src
git clone https://github.com/stereolabs/zed-ros-wrapper.git
git clone https://github.com/stereolabs/zed-ros-interfaces.git

#Installing 
echo "Compiling"
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
