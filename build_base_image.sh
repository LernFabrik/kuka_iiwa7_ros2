#!/bin/bash -e

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/docker/print_color.sh
DOCKER_DIR="${ROOT}/docker"


function usage(){
    print_info "Usage: run_env.sh" 
    print_info "Copyright (c) 2022, IWT Wirtschaft und Technik GmbH."
    print_info "Author: Vishnuprasad Prachandabhanu"
}

TARGET_IMAGE="$1"
ROS_LAYER="$1"
DEPENDENCY_LAYER="$1"
USE_LAYER="$1"

ON_EXIT=()
function cleanup {
    for command in "${ON_EXIT[@]}"
    do
        $command &>/dev/null
    done
}
trap cleanup EXIT

TARGET_IMAGE_IDS=(${TARGET_IMAGE//./ })
IMAGE_IDS=(${TARGET_IMAGE_IDS[@]})

print_info "$IMAGE_IDS"