colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
source install/setup.sh && \
ros2 run db3tomp4 db3tomp4_node && \