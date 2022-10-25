#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/docker/print_color.sh

function usage(){
    print_info "Usage: run_env.sh" 
    print_info "Copyright (c) 2022, IWT Wirtschaft und Technik GmbH."
    print_info "Author: Vishnuprasad Prachandabhanu"
}

usage
# Read and parse config file if exists
#
# CONFIG_IMAGE_KEY (string, can be empty)

if [[ -f "${ROOT}/.ros_common-config" ]]; then
    . "${ROOT}/.ros_common-config"
fi

ROS_DEV_DIR="$1"
if [[ -z "$ROS_DEV_DIR" ]]; then
    ROS_DEV_DIR="$HOME/ros_ws/colcon_kuka_ws"
    if [[ ! -d "$ROS_DEV_DIR" ]]; then
        ROS_DEV_DIR=$(pwd)
    fi
    print_info "colcon_kuka_ws is not specified, assuming $ROS_DEV_DIR"
else
    shift 1
fi

ON_EXIT=()

function cleanup {
    for command in "${ON_EXIT[@]}"
    do
        $command
    done
}

trap cleanup EXIT

pushd . >/dev/null
cd $ROOT
ON_EXIT+=("popd")

# Prevent running as root
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi

# Check if git-lfs is installed.
if [[ -z "$(git lfs)" ]] ; then
    print_error "git-lfs is not insalled. Please make sure git-lfs is installed before you clone the repo."
    exit 1
fi

PLATFORM="$(uname -m)"
BASE_NAME="ros2_kuka_$PLATFORM"
CONTAINER_NAME="$BASE_NAME-container"

# # Remove any exited containers.
# if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
#     docker rm $CONTAINER_NAME > /dev/null
# fi

# # Re-use existing container
# # Re-use existing container.
# if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
#     print_info "Attaching to running container: $CONTAINER_NAME"
#     docker exec -i -t -u admin --workdir /ros_ws/colcon_kuka_ws $CONTAINER_NAME /bin/bash $@
#     exit 0
# fi

# Build image
BASE_IMAGE=$PLATFORM
ROS_IMAGE=humble
DEP_IMAGE=dep
BASE_IMAGE_KEY=user

print_info "Building $BASE_IMAGE_KEY base as image: $BASE_NAME using key $BASE_IMAGE_KEY"
$ROOT/build_base_image.sh $BASE_IMAGE $ROS_IMAGE $DEP_IMAGE $BASE_IMAGE_KEY