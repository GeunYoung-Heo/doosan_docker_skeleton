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
        ffmpeg \
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
        ros-${ROS_DISTRO}-moveit-configs-utils \
 && rm -rf /var/lib/apt/lists/*

# --- rviz2 stub package ---
# Real rviz2 cannot be installed: ros-humble-rviz2 depends on
# ros-humble-rviz-ogre-vendor which requires libfreetype6-dev, and
# nvcr.io/nvidia/isaac-sim:4.5.0 holds libfreetype6 at an incompatible
# version. Isaac Sim replaces rviz for visualization.
#
# However, Doosan's official launch files (e.g. start.launch.py) hard-code
# a `Node(package="rviz2")` spawn with no conditional. Without at least the
# rviz2 *package metadata*, launch crashes at the OpaqueFunction stage and
# takes move_group + controllers down with it.
#
# This stub registers a minimal rviz2 package in ament_index and installs a
# no-op shell script as the executable. The launch system finds and spawns
# it successfully; it just sleeps instead of rendering.
RUN mkdir -p /opt/ros/${ROS_DISTRO}/share/ament_index/resource_index/packages \
 && touch /opt/ros/${ROS_DISTRO}/share/ament_index/resource_index/packages/rviz2 \
 && mkdir -p /opt/ros/${ROS_DISTRO}/share/rviz2 \
 && printf '<?xml version="1.0"?>\n<package format="3">\n  <name>rviz2</name>\n  <version>0.0.0</version>\n  <description>Stub — Isaac Sim replaces rviz2</description>\n  <maintainer email="stub@stub">stub</maintainer>\n  <license>Apache-2.0</license>\n</package>\n' \
      > /opt/ros/${ROS_DISTRO}/share/rviz2/package.xml \
 && mkdir -p /opt/ros/${ROS_DISTRO}/lib/rviz2 \
 && printf '#!/bin/bash\necho "[rviz2-stub] Isaac Sim replaces rviz2 in this container. Sleeping."\nwhile true; do sleep 86400; done\n' \
      > /opt/ros/${ROS_DISTRO}/lib/rviz2/rviz2 \
 && chmod +x /opt/ros/${ROS_DISTRO}/lib/rviz2/rviz2

# --- rosdep init ---
RUN rosdep init || true && rosdep update

# --- workspace skeleton ---
RUN mkdir -p ${WS}/src ${WS}/build ${WS}/install ${WS}/log
WORKDIR ${WS}

# --- interactive shell niceties: colored prompt, ls/grep colors, ROS sourcing ---
# Applied to /root/.bashrc so every `docker exec -it ... bash` gets colors
# without needing a login shell.
RUN cat >> /root/.bashrc <<'BASHRC'

# --- doosan_isaac interactive shell setup ---
export TERM=${TERM:-xterm-256color}
export CLICOLOR=1
export LS_COLORS="${LS_COLORS:-rs=0:di=01;34:ln=01;36:mh=00:pi=40;33:so=01;35:do=01;35:bd=40;33;01:cd=40;33;01:or=40;31;01:mi=00:su=37;41:sg=30;43:ca=30;41:tw=30;42:ow=34;42:st=37;44:ex=01;32}"
alias ls='ls --color=auto'
alias ll='ls -lah --color=auto'
alias la='ls -a --color=auto'
alias grep='grep --color=auto'
alias egrep='egrep --color=auto'
alias fgrep='fgrep --color=auto'
PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

# ROS 2 sourcing (safe to re-source)
if [ -f /opt/ros/${ROS_DISTRO:-humble}/setup.bash ]; then
  source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
fi
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
BASHRC

# --- bootstrap + entrypoint scripts (external files, easier to debug than heredoc) ---
COPY bootstrap_ws.sh /usr/local/bin/bootstrap_ws.sh
COPY entrypoint.sh   /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/bootstrap_ws.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
