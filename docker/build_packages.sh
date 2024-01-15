!/bin/bash

rosdep install --from-paths src --ignore-src -r -y 
source /opt/ros/noetic/setup.bash 
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
catkin build -j $(($(nproc) - 2))