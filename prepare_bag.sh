#!/bin/bash

extract_to=$1
curr_dir=${pwd}

cd ${extract_to}

wget https://aws-robomaker-sample-resources.s3-us-west-2.amazonaws.com/rosbags/jb_presentation_stream.tar.gz 
tar -xvzf jb_presentation_stream.tar.gz
rm jb_presentation_stream.tar.gz

cd ${curr_dir}
