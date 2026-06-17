##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=jazzy
FROM osrf/ros:jazzy-desktop-full
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
#POSIX standards-compliant default locale. Only strict ASCII characters are valid, extended to allow the basic use of UTF-8
ENV LANG C.UTF-8 
ENV LC_ALL C.UTF-8
ENV LC_ALL=C
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-testing && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-jazzy-ur \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-hardware-interface \
    ros-jazzy-transmission-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ur-description \
    ros-jazzy-ros-testing \
    ros-jazzy-moveit \
    ros-jazzy-moveit-core \
    ros-jazzy-moveit-common \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-ros \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-rviz-visual-tools \
    ros-jazzy-graph-msgs \
    autoconf \
    automake \
    python3-pip \
    python3-serial && \
    rm -rf /var/lib/apt/lists/*

 #   libltdl-dev \
  #  libtool \
  # ros-jazzy-ros-testing \ hinzugefügt damit topicbaseros2control build in jazzy geht
RUN rosdep init || true && rosdep update

RUN apt-get update && apt-get install --no-install-recommends -y \
    dirmngr gnupg2 lsb-release can-utils iproute2\
    apt-utils bash nano aptitude util-linux \
    htop git tmux sudo wget gedit bsdmainutils \
    pip && \
    rm -rf /var/lib/apt/lists/*

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=robot
ARG PASSWORD=robot
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=8
ARG VIDEO_GID=44
ENV ROS_DOMAIN_ID=${DOMAIN_ID}
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}

RUN groupadd -o -g "$GID" "$USER"  && \
    useradd -o -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    groupadd realtime && \
    groupmod -o -g ${VIDEO_GID} video && \
    usermod -aG video "$USER" && \
    usermod -aG dialout "$USER" && \
    usermod -aG realtime "$USER" && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 User Dependecies                         ##
##############################################################################

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
# hinzugefügt für topicbasedros2control
RUN rosdep update

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

CMD /bin/bash

