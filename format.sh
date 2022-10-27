#! /usr/bin/env sh
# reference: https://github.com/PointCloudLibrary/pcl/blob/master/.dev/format.sh
format(){
    local whitelist="iwtros2_launch"

    local formatter="${1}"

    if [ ! -f "${formatter}" ]; then
        echo "Could not find a clang-format. Please specify one as the first argument"
        exit 166
    fi


    for dir in ${whitelist}; do
        path=${dir}
        find ${path} -type f -iname *.[ch] -o -iname *.[ch]pp -o -iname *.[ch]xx \
            -iname *.cu | xargs -n1 ${formatter} -i -style=file
    done

}

format $@