# HSR packages (hsrb_common, hsrb_rosnav, tmc_database) on ROS 2 Humble.
ARG BASE_IMAGE=osrf/ros:humble-desktop-full
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/hsr_ws
COPY src /opt/hsr_ws/src

# --ignore-src: local package.xml names are not apt keys.
# Isolated install (not --symlink-install): ament local_setup.bash is a
# symlink into build/ under symlink-install, and we delete build/.
RUN . /opt/ros/humble/setup.sh \
    && apt-get update \
    && (rosdep init || true) \
    && rosdep update \
    && rosdep install --from-paths . -y --ignore-src --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.sh \
    && colcon build \
    && rm -rf /opt/hsr_ws/build /opt/hsr_ws/log

# Live checkout is the bind-mount at /ws. Image only keeps the install overlay.
RUN rm -rf /opt/hsr_ws/src

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/cyclonedds_profile.xml /opt/cyclonedds_profile.xml
RUN chmod +x /entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=96 \
    CYCLONEDDS_URI=file:///opt/cyclonedds_profile.xml

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
