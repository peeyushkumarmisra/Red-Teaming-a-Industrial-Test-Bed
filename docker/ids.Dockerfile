# =============================================================================
#  ids.Dockerfile - image for the intrusion detection system
# -----------------------------------------------------------------------------
#  A small ROS 2 Jazzy base image with only what the IDS needs: a compiler, Pinocchio
#  (rigid-body dynamics), yaml-cpp, gtest, Cyclone DDS and MCAP rosbag support.
#  No Gazebo: the IDS must not depend on the simulator it watches.
#  Build inside: `colcon build --packages-up-to ids_ros`.
# =============================================================================

# ROS 2 Jazzy (Ubuntu 24.04) without desktop tools.
FROM ros:jazzy-ros-base

# Non-root user whose IDs match a typical host user, so mounted files keep sane owners.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# Toolchain and the libraries the IDS links against.
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo build-essential cmake git \
      python3-colcon-common-extensions python3-rosdep \
      libyaml-cpp-dev libgtest-dev libeigen3-dev \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-pinocchio \
      ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Create the dev user, give it sudo, and prepare /ws for the mounted workspace.
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
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
