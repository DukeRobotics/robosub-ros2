original_cwd=$(pwd)

if [ -n "$1" ]; then
    cd /root/dev/robosub-ros2/onboard
    colcon build --symlink-install --packages-select "$1"
    source install/setup.bash
else
    cd /root/dev/robosub-ros2/core
    colcon build --symlink-install --executor sequential
    cd /root/dev/robosub-ros2/onboard
    colcon build --symlink-install --executor sequential
    source install/setup.bash
fi

source ~/.bashrc
cd "$original_cwd"
