# Use the official ROS image as the base image
FROM ros:noetic-robot-focal

RUN apt-get update && apt-get install -y ros-noetic-husky-* \
    && apt-get install -y ros-noetic-ddynamic-reconfigure \
    && apt-get install -y ros-noetic-imu-filter-madgwick \
    && apt-get install -y ros-noetic-imu-transformer

# Install additional packages
RUN apt-get install -y git wget jq nano

# Add ROS repository and install dependencies
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y python3-catkin-tools python3-pip 

COPY ./ /robot_sim
WORKDIR /robot_sim

# Build Packages
SHELL ["/bin/bash", "-c"] 
RUN chmod +x ./docker/build_packages.sh && \
    ./docker/build_packages.sh

# Keep only the install folder by excluding them
RUN find . -maxdepth 1 ! -name 'install' ! -name 'repos.yml' ! -name '.' -exec rm -rf {} +