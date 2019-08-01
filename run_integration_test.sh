#!/bin/bash
set -v

if [ -f "./travis" ]; then
    export TRAVIS=true
fi

if [ "$TRAVIS" == true ]; then
    pip3 install massedit
else
    sudo -H pip3 install massedit
fi

extract_to=/tmp
extracted_bag_name="${extract_to}/new_format_short"
if [ ! -d "$extracted_bag_name" ]; then
    echo "Downloading and extracting bag file"
    bash prepare_bag.sh ${extract_to}
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
workspace=${SCRIPT_DIR}
source /opt/ros/dashing/local_setup.bash

if [ ! -f "./install/local_setup.bash" ]; then
    echo "Building workspace"
    if [ "$TRAVIS" != true ]; then
        rosws update
        if [ -d "src/deps/kinesisvideo-encoder-ros2/kinesis_video_msgs" ]; then
            rm -rf "src/deps/kinesisvideo-encoder-ros2/kinesis_video_msgs"
        fi
        colcon build
    fi
fi

if [ "$TRAVIS" != true ]; then
    upstream_workspace=.
else
    upstream_workspace=/root/upstream_ws
    . .creds
    cp "${upstream_workspace}/src/src/deps/kinesisvideo-ros2/kinesis_video_streamer/config/kvs_log_configuration" "/tmp/kvs_log_configuration"
fi
source ${upstream_workspace}/install/local_setup.bash

echo "Running test_kinesis.py"
python3 test_kinesis.py -v ${extracted_bag_name}
exit_code=$?
cat /tmp/rekognition_results
exit ${exit_code}
