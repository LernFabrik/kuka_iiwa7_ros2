#!/bin/bash -e

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/print_color.sh
DOCKER_DIR="${ROOT}/../docker"

DOCKER_SEARCH_DIRS=(${DOCKER_DIR})

function usage(){
    print_info "Usage: run_env.sh"
    print_info "Copyright (c) 2022, IWT Wirtschaft und Technique GmbH."
    print_info "Author: Vishnuprasad Prachandabhanu"
}

BASE_IMAGE="$1"
ROS_LAYER="$2"
DEPENDENCY_LAYER="$3"
USER_LAYER="$4"

ON_EXIT=()
function cleanup {
    for command in "${ON_EXIT[@]}"
    do
        $command &>/dev/null
    done
}
trap cleanup EXIT

DOCKERFILES=()
DOCKERFILE_CONTEXT_DIRS=()

DOCKERFILE="${DOCKER_SEARCH_DIRS}/Dockerfile.${BASE_IMAGE}"
if [[ -f "${DOCKERFILE}" ]]; then
    DOCKERFILES+=(${DOCKERFILE})
    DOCKERFILE_CONTEXT_DIRS+=(${DOCKER_SEARCH_DIRS})
fi
DOCKERFILE="${DOCKER_SEARCH_DIRS}/Dockerfile.${ROS_LAYER}"
if [[ -f "${DOCKERFILE}" ]]; then
    DOCKERFILES+=(${DOCKERFILE})
    DOCKERFILE_CONTEXT_DIRS+=(${DOCKER_SEARCH_DIRS})
fi
DOCKERFILE="${DOCKER_SEARCH_DIRS}/Dockerfile.${DEPENDENCY_LAYER}"
if [[ -f "${DOCKERFILE}" ]]; then
    DOCKERFILES+=(${DOCKERFILE})
    DOCKERFILE_CONTEXT_DIRS+=(${DOCKER_SEARCH_DIRS})
fi
DOCKERFILE="${DOCKER_SEARCH_DIRS}/Dockerfile.${USER_LAYER}"
if [[ -f "${DOCKERFILE}" ]]; then
    DOCKERFILES+=(${DOCKERFILE})
    DOCKERFILE_CONTEXT_DIRS+=(${DOCKER_SEARCH_DIRS})
fi

# for DOCKERFILE in ${DOCKERFILES[@]}; do
#     print_info "Selected Docker file: $DOCKERFILE"
# done

# BUILD_ARGS+=("--build-arg" "USERNAME="admin"")
# PLATFORM="$(uname -m)"
# if [[ $PLATFORM == "x86_64" ]]; then
#     if type nvidia-smi &>/dev/null; then
#         GPU_ATTACHED=(`nvidia-smi -a | grep "Attached GPUs"`)
#         if [ ! -z $GPU_ATTACHED ]; then
#             BUILD_ARGS+=("--build-arg" "HAS_GPU="true"")
#         fi
#     fi
# fi
PLATFORM="$(uname -m)"

for (( i=0; i<${#DOCKERFILES[@]}; i++ )); do
    DOCKERFILE=${DOCKERFILES[i]}
    print_info "Selected Docker file: $DOCKERFILE"

    BASE_IMAGE_ARG=
    IMAGE_NAME=
    if [[ $i -eq 0 ]]; then
        IMAGE_NAME="$PLATFORM"
        BASE_IMAGE_ARG=""
    fi
    if [[ $i -eq 1 ]]; then
        IMAGE_NAME="$PLATFORM.humble"
        BASE_IMAGE_ARG="--build-arg BASE_IMAGE=${PLATFORM}"
    fi

    if [[ $i -eq 2 ]]; then
        IMAGE_NAME="$PLATFORM.humble.dep"
        BASE_IMAGE_ARG="--build-arg BASE_IMAGE=${PLATFORM}.humble"
    fi

    if [[ $i -eq 3 ]]; then
        IMAGE_NAME="$PLATFORM.humble.dep.user"
        BASE_IMAGE_ARG="--build-arg BASE_IMAGE=${PLATFORM}.humble.dep"
    fi

    print_info "Selected Docker file: $DOCKERFILE image name: $IMAGE_NAME and build argument: ${BASE_IMAGE_ARG}"
    docker build -f ${DOCKERFILE} --network host -t ${IMAGE_NAME} ${BASE_IMAGE_ARG} .
done
