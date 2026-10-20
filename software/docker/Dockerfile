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
    && rm -rf /var/lib/apt/lists/*

# The noble base image already provides the "ubuntu" user (uid/gid 1000).
RUN echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
ENV WS=/home/${USERNAME}/master-s-thesis/software/ros2_ws

RUN rosdep update \
    && echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"' >> ~/.bashrc \
    && echo 'cd "$WS"' >> ~/.bashrc

WORKDIR /home/${USERNAME}/master-s-thesis
CMD ["sleep", "infinity"]
