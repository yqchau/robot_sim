# Use the official ROS image as the base image
FROM ros:noetic-robot-focal

# Install ROS desktop packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-noetic-desktop=1.5.0-1*

RUN apt-get install -y ros-noetic-husky-* \
        && ros-noetic-ddynamic-reconfigure \
        && ros-noetic-imu-filter-madgwick \
        && ros-noetic-imu-transformer

# Install additional packages
RUN apt-get install -y git wget jq nano

# Add ROS repository and install dependencies
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y python3-catkin-tools python3-pip 

RUN rosdep install --from-paths src --ignore-src -r -y \
 && source /opt/ros/noetic/setup.bash \ 
 && catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False \
 && catkin build -j $(($(nproc) - 2))

# Keep only the install folder & repos.yml by excluding them
RUN find . -maxdepth 1 ! -name 'install' ! -name '.' -exec rm -rf {} +
