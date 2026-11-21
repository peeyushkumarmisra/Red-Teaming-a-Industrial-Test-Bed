# ==================================================================================================
# robot.Dockerfile - image for the simulated robot cell
# --------------------------------------------------------------------------------------------------
# ROS 2 Jazzy desktop with Gazebo Harmonic, ros2_control and gz_ros2_control, Pinocchio, xacro,
# the ROS-Gazebo bridge, MCAP recording, and the Python tools for the planner, the attacker and the
# tests (ikpy pinned to 4.5.0).
# Software rendering (mesa) lets Gazebo run headless or show a GUI through an Xserver on Windows.
# ==================================================================================================

# ROS2 Jazzy (Ubuntu 24.04) without desktop tools.
FROM osrf/ros:jazzy-desktop

# Non-root user whole IDS match a typical host user, hence mounted files keeps same owners
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# Simulation, control and Python test dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo build-essential cmake git \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    python3-flake8 python3-pytest python3-pandas python3-matplotlib python3-yaml \
    libyaml-cpp-dev libgtest-dev libeigen3-dev \
    mesa-utils libgl1-mesa-dri \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-pinocchio \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control ros-jazzy-ros-gz \
    ros-jazzy-xacro ros-jazzy-realtime-tools \
    ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# ikpy (inverse kinematics for the planner) is pinned so the IK seeds give the same poses everywhere
RUN pip3 install --no-cache-dir --break-system-packages ikpy==4.5.0

# Creating the dev user, giving it sudo and preparing /ws for the mounted workspace
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /ws/src /ws/build /ws/install /ws/log /ws/data \
    && chown -R ${USERNAME}:${USERNAME} /ws \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

# Every command runs with ROS sourced (see entrypoint.sh).
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENV DEBIAN_FRONTEND=dialog
USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]
