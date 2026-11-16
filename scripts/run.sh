#!/bin/bash
# Start a throwaway container with what real hardware needs: host network (DDS and
# SocketCAN interfaces), host IPC (Fast DDS shared memory), /dev (USB adapters), and
# SYS_NICE (real-time priority for ros2_control_node).
#   bash scripts/run.sh                 -> shell in ros2_control_image:local
#   bash scripts/run.sh <image> [cmd]   -> other image / command
IMAGE="${1:-ros2_control_image:local}"
shift || true
exec docker run -it --rm \
    --network host --ipc host --privileged \
    --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
    -v /dev:/dev \
    "$IMAGE" "${@:-bash}"
