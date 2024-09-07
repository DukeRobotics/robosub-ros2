original_cwd=$(pwd)
cd /root/dev/robosub-ros2/onboard
colcon build --symlink-install --executor sequential
source install/setup.bash
cd "$original_cwd"
