# UR3 Writer Package - ROS 2 Humble & MoveIt 2

Package điều khiển robot UR3 thực hiện vẽ chữ 'D' trong môi trường mô phỏng Gazebo và RViz2 bằng MoveIt 2 Cartesian Path.

## 1. Yêu cầu hệ thống
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Sim / Ignition
- MoveIt 2 (`moveit_ros_planning_interface`)

## 2. Cấu trúc Package
- `src/ur3_writer_node.cpp`: Node C++ lập kế hoạch đường đi Cartesian, phát Marker hiển thị nét vẽ chữ D và thu gọn cánh tay.
- `launch/ur3_writer.launch.py`: File launch khởi động đồng thời mô phỏng UR3, RViz2 và node điều khiển.
- `rviz/ur3_writer.rviz`: Cấu hình giao diện RViz2 lưu sẵn hiển thị Marker.

## 3. Hướng dẫn biên dịch và chạy
```bash
cd ~/workspaces/ur_gz
colcon build --packages-select ur3_writer
source install/setup.bash
ros2 launch ur3_writer ur3_writer.launch.py
