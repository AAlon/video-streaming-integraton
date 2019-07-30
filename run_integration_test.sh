#!/bin/bash

extracted_bag_name="new_format_short"
if [ ! -d "$extracted_bag_name" ]; then
    echo "Downloading and extracting bag file"
    bash prepare_bag.sh
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
workspace=${SCRIPT_DIR}
source /opt/ros/dashing/setup.bash

if [ ! -d "./install" ]; then
    echo "Building workspace"
    rosws update
    colcon build
fi

echo "Running test_kinesis.py"
source ${workspace}/install/local_setup.bash
python3 test_kinesis.py
