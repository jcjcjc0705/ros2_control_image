# ros2_control_image: everything a robot image needs that isn't specific to one robot.
#   ros_jazzy_base_image -> ros2_control_image (this) -> <robot> image
# Needs a base whose .bashrc sources $ROS_UNDERLAY (1.1.0 or later).
ARG BASE_IMAGE=ghcr.io/jcjcjc0705/ros_jazzy_base_image:latest
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

# ── ros2_control, the standard controllers/broadcasters, bring-up tools ─────
RUN apt-get update && apt-get install -y \
        can-utils \
        iproute2 \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-xacro && \
    rm -rf /var/lib/apt/lists/*

# ── Shared packages (hw_transport, mit_msgs, mit_controller, robot_bringup) ─
# Built into an underlay at /opt/ros2_control_ws. A robot image builds its own
# underlay on top of this one, and $ROS2_WS (/workspaces) stays empty for
# bind-mounted feature packages.
ENV ROS2_CONTROL_WS=/opt/ros2_control_ws
COPY ros2_ws/src ${ROS2_CONTROL_WS}/src
RUN apt-get update && \
    rosdep install --from-paths ${ROS2_CONTROL_WS}/src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/* && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ROS2_CONTROL_WS} && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log

# The top-most underlay; the base image's .bashrc (and so its entrypoint) sources
# it. A robot image overrides this with its own install/ directory, whose
# setup.bash chains back to this one. ENTRYPOINT, CMD and WORKDIR come from the
# base image.
ENV ROS_UNDERLAY=${ROS2_CONTROL_WS}/install
