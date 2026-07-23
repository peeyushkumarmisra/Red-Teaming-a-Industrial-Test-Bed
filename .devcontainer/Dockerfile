FROM ubuntu:24.04

# Prevent interactive prompts (like timezone selections) from freezing your manual installs
ENV DEBIAN_FRONTEND=noninteractive

# Install bare-minimum tools needed to fetch and install things later
RUN apt-get update && apt-get install -y \
    # WHAT: Command-line tool to transfer data using URLs.
    # WHY: Used to download GPG keys and setup repository configurations.
    curl \
    # WHAT: GNU Privacy Guard (v2), an encryption and signing tool.
    # WHY: Needed to securely verify the authenticity of the ROS 2 repository keys before downloading packages.
    gnupg2 \
    # WHAT: Linux Standard Base repository info tool.
    # WHY: Helps scripts automatically detect your exact Ubuntu version name (e.g., "noble" for 24.04) to fetch the right packages.
    lsb-release \
    # WHAT: "Superuser Do" command privilege manager.
    # WHY: Allows scripts or programs running inside the container to execute tasks with administrative permissions if required.
    sudo \
    # WHAT: Utilities for managing independent software repositories (PPAs).
    # WHY: Gives you the `add-apt-repository` command, which is often required to add third-party software sources.
    software-properties-common \
    # WHAT: Network utility to download files via HTTP, HTTPS, and FTP.
    # WHY: Similar to curl; it's a standard fallback used by many automated installation scripts to fetch files.
    wget \
    # WHAT: Git version control system.
    # WHY: Essential for cloning ROS 2 source code repositories, custom packages, or workspace dependencies.
    git \
    # WHAT: A meta-package containing GCC (C/C++ compiler), make, and other core compilation utilities.
    # WHY: ROS 2 packages heavily rely on C++; you need this to build and compile nodes from source.
    build-essential \
    # WHAT: A collection of lightweight, graphical X11 test applications (like xeyes or xclock).
    # WHY: Crucial for testing and troubleshooting GUI/graphical forwarding from your Docker container to your host machine.
    x11-apps \
    && add-apt-repository universe -y \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    libeigen3-dev \
    python3-colcon-common-extensions ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Automatically source the ROS 2 environment variable when you open a terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Revert to standard terminal behavior
ENV DEBIAN_FRONTEND=dialog