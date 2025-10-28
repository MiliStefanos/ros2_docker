FROM ros:humble
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && apt-get install -y \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL=/bin/bash

# Add ROS 2 sources and key
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list
RUN apt-get update

# Install ROS 2 desktop and needed tools
RUN apt-get install -y \
    ros-humble-ros-base \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    libeigen3-dev \
    libyaml-cpp-dev \
    liburdfdom-tools \
    && rm -rf /var/lib/apt/lists/*

# Set workspace
WORKDIR /ros2_ws
COPY ./src ./src

# Build
RUN . /opt/ros/humble/setup.sh \
    && rosdep update && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build

# Default launch command
ENTRYPOINT ["/bin/bash"]
USER $USERNAME