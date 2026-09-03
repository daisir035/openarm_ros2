FROM osrf/ros:humble-desktop-full

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN . /opt/ros/humble/setup.sh && \
    apt update && \
    apt install nlohmann-json3-dev python3-vcstool -y && \
    mkdir --parents ~/ros2_ws/src && \
    git clone --branch main https://github.com/daisir035/openarm_ros2.git ~/ros2_ws/src/openarm_ros2 && \
    cd ~/ros2_ws/src && \
    vcs import < openarm_ros2/openarm.repos && \
    cd ~/ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
