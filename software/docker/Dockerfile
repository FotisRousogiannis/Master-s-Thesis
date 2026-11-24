# Development image: ROS 2 Jazzy + Gazebo Harmonic + ros2_control + MoveIt 2.
# Same image for simulation and for driving the real arm from the workstation.
FROM osrf/ros:jazzy-desktop

ARG USERNAME=ubuntu
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-moveit \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui \
        python3-colcon-common-extensions \
        python3-pytest \
        python3-pip \
        git sudo bash-completion nano mesa-utils \
        wmctrl \
        openssh-client \
    && rm -rf /var/lib/apt/lists/*

# Packages added later go in this small layer, NOT in the big one above: changing the big RUN makes Docker
# download ROS, Gazebo and MoveIt again (30 min); changing this one only installs what is listed here.
#   python3-psutil: network interfaces for arm_link discovery (ADR-0022) · iproute2: the `ip` command
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-psutil \
        iproute2 \
    && rm -rf /var/lib/apt/lists/*

# topic_based_ros2_control (hardware:=real, ros2_control <-> topics of the Pi, ADR-0024) has no Jazzy binary
# (apt: "Unable to locate package", 2026-11-24): built from source, pinned to the release 0.3.0 whose behaviour
# ADR-0024 describes. Installed as an overlay in /opt/topic_based_ros2_control, sourced after /opt/ros/jazzy.
ARG TOPIC_BASED_VERSION=0.3.0
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-angles \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --depth 1 --branch ${TOPIC_BASED_VERSION} \
        https://github.com/PickNikRobotics/topic_based_ros2_control.git /tmp/tb_ws/src/topic_based_ros2_control \
    && cd /tmp/tb_ws && . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --install-base /opt/topic_based_ros2_control \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/tb_ws

# The noble base image already provides the "ubuntu" user (uid/gid 1000).
RUN echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
ENV WS=/home/${USERNAME}/master-s-thesis/software/ros2_ws

RUN rosdep update \
    && echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo 'source /opt/topic_based_ros2_control/setup.bash' >> ~/.bashrc \
    && echo '[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"' >> ~/.bashrc \
    && echo 'cd "$WS"' >> ~/.bashrc

WORKDIR /home/${USERNAME}/master-s-thesis
CMD ["sleep", "infinity"]
