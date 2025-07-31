# ============================================================================
# ROS 2 Jazzy Dev Environment (Ubuntu 24.04)
# References:
#   * ROS 2 Jazzy installation guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#   * TurtleBot 4 simulator manual: https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html

FROM ubuntu:24.04

## Set non‑interactive frontend and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

## Install base packages, locales and tzdata in a single layer
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        locales tzdata \
        curl gnupg2 lsb-release sudo wget software-properties-common \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

## Add the ROS 2 APT repository and key
RUN set -ex; \
    apt-get update && apt-get install -y --no-install-recommends curl gnupg2 && \
    if curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key >/dev/null; then \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list ; \
    fi && \
    rm -rf /var/lib/apt/lists/*

## Update system and install ROS 2 and simulation packages
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-jazzy-nav2-bringup \
        ros-jazzy-turtlebot4-simulator \
        ros-jazzy-teleop-twist-keyboard \
        python3-colcon-common-extensions \
        ros-jazzy-ament-cmake \
        python3-argcomplete \
        python3-pip \
        python3-rosdep \
        ros-dev-tools \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

## Initialize rosdep (ignore errors if it already exists)
RUN rosdep init || true && rosdep update || true

## Set environment variables for ROS
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:${PATH}

ARG USERNAME=tester
ARG USER_UID=1050
ARG USER_GID=1050
ARG REPO_DIR=nav2-performance-testing

RUN groupadd --gid $USER_GID $USERNAME \
 && useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
RUN mkdir -p /home/$USERNAME/REPO_DIR/
WORKDIR /home/$USERNAME/$REPO_DIR/ros2_ws

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
 && echo "if [ -f /home/$USERNAME/REPO_DIR/ros2_ws/install/setup.bash ]; then source /home/tester/$REPO_DIR/ros2_ws/install/setup.bash; fi" >> /home/tester/.bashrc \
 && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/tester/.bashrc

