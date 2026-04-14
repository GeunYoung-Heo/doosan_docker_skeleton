FROM nvcr.io/nvidia/isaac-sim:4.5.0

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WS=/ros2_ws
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y

# --- ROS 2 Humble apt repository ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common curl gnupg lsb-release ca-certificates \
 && add-apt-repository universe \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list \
 && rm -rf /var/lib/apt/lists/*

# --- ROS 2 Humble + dependencies ---
# NOTE: Use ros-base (not desktop) and skip rviz/visualization packages.
# nvcr Isaac Sim base image holds libfreetype6 at a version that conflicts with
# rviz-ogre-vendor's libfreetype6-dev requirement. Visualization is handled by
# Isaac Sim, not rviz, so this is a non-loss.
RUN apt-get update && apt-get install -y --no-install-recommends \
        wget git build-essential cmake \
        libpoco-dev libyaml-cpp-dev dbus-x11 \
        python3-rosdep python3-colcon-common-extensions \
        liburdfdom-tools \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-control-msgs \
        ros-${ROS_DISTRO}-realtime-tools \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-example-interfaces \
        ros-${ROS_DISTRO}-moveit-ros-planning \
        ros-${ROS_DISTRO}-moveit-ros-planning-interface \
        ros-${ROS_DISTRO}-moveit-kinematics \
        ros-${ROS_DISTRO}-moveit-planners-ompl \
        ros-${ROS_DISTRO}-moveit-simple-controller-manager \
        ros-${ROS_DISTRO}-moveit-ros-move-group \
 && rm -rf /var/lib/apt/lists/*

# --- rosdep init ---
RUN rosdep init || true && rosdep update

# --- workspace skeleton ---
RUN mkdir -p ${WS}/src ${WS}/build ${WS}/install ${WS}/log
WORKDIR ${WS}

# --- bootstrap + entrypoint scripts (external files, easier to debug than heredoc) ---
COPY bootstrap_ws.sh /usr/local/bin/bootstrap_ws.sh
COPY entrypoint.sh   /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/bootstrap_ws.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
