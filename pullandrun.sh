git pull
cd puzzlebot_ws
colcon build
source install/setup.bash
sudo service nvargus-daemon restart
ros2 run puzzlebot camera_segmentation_node