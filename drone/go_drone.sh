#!/bin/bash
echo "Khởi động Hệ thống SLAM (Google Cartographer) + Mô hình F450 3D..."

# 0. DỌN DẸP
sudo killall -9 socat 2>/dev/null
sudo killall -9 ydlidar_ros2_driver_node 2>/dev/null
sudo killall -9 cartographer_node 2>/dev/null
sudo killall -9 cartographer_occupancy_grid_node 2>/dev/null
sudo killall -9 robot_state_publisher 2>/dev/null
fuser -k -9 8888/udp
fuser -k -9 8889/udp
fuser -k -9 8890/udp

sleep 1

# 1. TẠO CẦU NỐI WIFI CHO LIDAR
gnome-terminal --title="1. Lidar Bridge" -- bash -c "sudo socat pty,link=/dev/ydlidar,raw udp4-listen:8889; exec bash"
sleep 5
sudo chmod 777 /dev/ydlidar

# 2. BẬT MẮT LIDAR
gnome-terminal --title="2. Lidar Driver" -- bash -c "source /opt/ros/humble/setup.bash; ros2 launch ydlidar_ros2_driver ydlidar_launch.py; exec bash"
sleep 4

# 3. BẬT CẦU NỐI IMU 
# BẮT BUỘC ĐỂ MẠCH ĐỨNG YÊN 3 GIÂY ĐỂ CALIBRATE!
gnome-terminal --title="3. IMU Bridge" -- bash -c "source /opt/ros/humble/setup.bash; python3 ~/imu_bridge.py; exec bash"
sleep 2

# ==========================================
# 4. ĐẨY MÔ HÌNH 3D URDF LÊN (THAY THẾ CHO TF STATIC CŨ)
# ==========================================
gnome-terminal --title="4. Robot Model" -- bash -c "source /opt/ros/humble/setup.bash; ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(cat ~/f450_realistic.urdf)\"; exec bash"
sleep 1

# ==========================================
#  5. GOOGLE CARTOGRAPHER CORE 
# ==========================================
gnome-terminal --title="5. Carto Core" -- bash -c "source /opt/ros/humble/setup.bash; ros2 run cartographer_ros cartographer_node -configuration_directory ~/cartographer_config -configuration_basename drone_2d.lua --ros-args -r scan:=/scan -r imu:=/imu/data; exec bash"
sleep 2

# ==========================================
#  6. CARTOGRAPHER GRID
# ==========================================
gnome-terminal --title="6. Carto Grid" -- bash -c "source /opt/ros/humble/setup.bash; ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p resolution:=0.05 -p publish_period_sec:=0.2; exec bash"

# ==========================================
# 7. RVIZ2
# ==========================================
gnome-terminal --title="7. RViz2" -- bash -c "source /opt/ros/humble/setup.bash; export QT_QPA_PLATFORM=xcb; ros2 run rviz2 rviz2 -d ~/f450.rviz 2>/dev/null; exec bash"
sleep 1

# ==========================================
# 8. ĐIỀU KHIỂN BAY
# ==========================================
gnome-terminal --title="8. Control" -- bash -c "source /opt/ros/humble/setup.bash; python3 ~/drone_controller.py; exec bash"

echo "HỆ THỐNG ĐÃ SẴN SÀNG! BAY THÔI!"
