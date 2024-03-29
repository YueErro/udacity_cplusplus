FROM osrf/ros:noetic-desktop-full

LABEL maintainer="Yue Erro <yue.trbj@gmail.com>"

ARG REPO_WS=/udacity_cplusplus_ws
RUN mkdir -p $REPO_WS/src
WORKDIR $REPO_WS

ARG IMAGE_NAME
ENV IMG_NAME=$IMAGE_NAME

ENV ROS_DISTRO noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    locales \
    dpkg \
    python3-rosdep \
    git \
    ssh \
    curl \
    wget \
    apt-utils \
    aptitude \
    g++ \
    gcc \
    openvpn \
    gnupg \
    bash-completion \
    vim \
    nano \
    tmux \
    psmisc \
    aptitude \
    build-essential \
    ccache \
    clang \
    clang-format \
    gdb \
    qtcreator \
    htop \
    man \
    meld \
    silversearcher-ag \
    terminator \
    tig \
    valgrind \
    docker \
    iputils-ping \
    ipython3 \
    python3-scipy \
    python3-vcstool \
    python3-wstool \
    python3-networkx \
    python3-pip  \
    python3-rosinstall \
    python3-catkin-tools \
    feh \
    ros-$ROS_DISTRO-rosbash \
    ros-$ROS_DISTRO-turtlebot3-gazebo \
    ros-$ROS_DISTRO-turtlebot3-teleop \
  && rm -rf /var/lib/apt/lists/* \
  && wget https://raw.githubusercontent.com/YueErro/udacity_cplusplus/master/Capstone-ROS-Node-simulated-in-Gazebo/docker/capstone_ros.rosinstall \
  && vcs import src < capstone_ros.rosinstall

RUN bash -c "source /opt/ros/noetic/setup.bash \
  && catkin build \
  && echo 'source $REPO_WS/devel/setup.bash' >> ~/.bashrc"

ENTRYPOINT ["bash"]
