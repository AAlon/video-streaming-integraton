# video-streaming-integraton

End to end testing including person detection via Rekognition. If the badge is green, it means we were able to encode & stream a `sensor_msgs::msg::Image` video all the way to Kinesis Video, analyze the results in Amazon Rekognition and read them back. 

**Media pipeline flow:**

`ros2 bag play` -> `h264_video_encoder` -> `kinesis_video_streamer` -> Amazon Kinesis Video Streams -> Amazon Rekognition 


**Data flow (Rekognition results):** 

Amazon Rekognition -> Amazon Kinesis Data Streams -> `kinesis_video_streamer` writes the results to topic `/rekognition/results`

* Travis CI: [![Build Status](https://travis-ci.org/AAlon/video-streaming-integraton.svg?branch=master)](https://travis-ci.org/AAlon/video-streaming-integraton)

### Local execution
To run locally on your development machine, clone this repository and invoke:

* `./run_integration_tests.sh` 

If running inside a minimal ROS2 environment (e.g. docker without wget, rosws, aws-cli etc.), or encountering missing dependencies issues - first run `./install_deps.sh` (with sudo if on your development machine).
