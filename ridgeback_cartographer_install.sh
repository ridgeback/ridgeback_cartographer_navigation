# Install wstool and rosdep.
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build

# Create a new workspace in 'ridgeback_cartographer_ws'.
mkdir ridgeback_cartographer_ws
cd ridgeback_cartographer_ws
wstool init src
cp -r ../ridgeback_cartographer_navigation ./src
git -C ./src clone https://github.com/ridgeback/ridgeback_desktop.git
git -C ./src clone https://github.com/ridgeback/ridgeback_simulator.git
git -C ./src clone https://github.com/ridgeback/ridgeback.git
cp src/ridgeback_cartographer_navigation/cartographer.rviz src/ridgeback_desktop/ridgeback_viz/rviz/

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
cd src/cartographer
git checkout e10650910e157bd68c55f4d1bc6603f25f89d293
cd ../cartographer_ros
git checkout 8895bfc96a599f23ce080cddfbaf5ce1c1bb7338
cd ../..

# Install deb dependencies.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
catkin_make_isolated --install --use-ninja \
  -DCMAKE_PREFIX_PATH="${PWD}/install_isolated;${CMAKE_PREFIX_PATH}"
source install_isolated/setup.bash
