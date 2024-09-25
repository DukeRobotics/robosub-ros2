original_cwd=$(pwd)
cd /root/dev/robosub-ros2/core
colcon build --symlink-install --executor sequential
cd /root/dev/robosub-ros2/onboard
colcon build --symlink-install --executor sequential
source install/setup.bash
source ~/.bashrc
cd "$original_cwd"
