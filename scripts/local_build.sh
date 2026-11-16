#!/bin/bash
# Build the image locally for the current architecture.
#   bash scripts/local_build.sh                              -> ros2_control_image:local
#   bash scripts/local_build.sh -t my:tag                    -> custom tag
#   bash scripts/local_build.sh -b ros_jazzy_base_image:local -> on a locally built base
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
TAG="ros2_control_image:local"
BASE=""
while [ "$#" -gt 0 ]; do
    case "$1" in
        -t) TAG="$2"; shift 2 ;;
        -b) BASE="$2"; shift 2 ;;
        *) echo "Usage: $0 [-t <tag>] [-b <base image>]"; exit 1 ;;
    esac
done
if [ -n "$BASE" ]; then
    # A local-only base can't be pulled, so no --pull here.
    docker build --build-arg BASE_IMAGE="$BASE" -t "$TAG" .
else
    docker build --pull -t "$TAG" .
fi
