#!/bin/bash

pip3 install massedit

extracted_bag_name="new_format_short"
if [ ! -d "$extracted_bag_name" ]; then
    echo "Downloading and extracting bag file"
    bash prepare_bag.sh
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
workspace=${SCRIPT_DIR}
source /opt/ros/dashing/local_setup.bash

if [ ! -f "./install/local_setup.bash" ]; then
    echo "Building workspace"
    rosws update
    if [ -d "src/deps/kinesisvideo-encoder-ros2/kinesis_video_msgs" ]; then
        rm -rf "src/deps/kinesisvideo-encoder-ros2/kinesis_video_msgs"
    fi
    colcon build
fi

echo "Running test_kinesis.py"
source ${workspace}/install/local_setup.bash
python3 test_kinesis.py -v
