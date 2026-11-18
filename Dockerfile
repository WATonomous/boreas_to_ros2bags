# Optional portable environment, including the camera decoder.
ARG BASE_IMAGE=ros:jazzy-ros-core
FROM ${BASE_IMAGE}
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap ros-jazzy-ros2bag \
    ros-jazzy-sensor-msgs ros-jazzy-tf2-msgs python3-numpy python3-pil \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /converter
COPY . .
ENV PYTHONDONTWRITEBYTECODE=1
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash; exec python3 \"$@\"", "bash"]
CMD ["-m", "boreas_to_ros2bag.convert", "--help"]
