FROM ros:noetic
 
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_HOSTNAME=localhost
 
WORKDIR /root
 
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libboost-dev \
    libopencv-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions \
    ros-noetic-tf-conversions \
    ros-noetic-random-numbers \
    ros-noetic-foxglove-bridge \
    python3-catkin-pkg \
    python3-rospkg \
    git \
&& rm -rf /var/lib/apt/lists/* \
&& \
    \
    # FAST keypoints lib for msckf_mono
    git clone https://github.com/uzh-rpg/fast && \
    cmake -S fast -B fast/build && \
    cmake --build fast/build -j$(nproc) && \
    cmake --install fast/build
 
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
 
ENTRYPOINT [ "/bin/bash" ]
