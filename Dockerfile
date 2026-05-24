FROM osrf/ros:humble-desktop-jammy

# Install workspace dependencies not in the desktop image
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-storage-default \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rmw-cyclonedds-cpp \
    libeigen3-dev \
    python3-pip \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# evo trajectory evaluation tool (uses matplotlib + tkinter for plots)
RUN pip3 install --no-cache-dir evo

# CycloneDDS is more reliable than FastDDS in containerised environments
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Copy workspace source and build
WORKDIR /ws
COPY src/ src/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build/

# Entrypoint sources both ROS2 and workspace overlays
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
