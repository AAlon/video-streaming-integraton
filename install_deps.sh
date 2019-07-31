#!/bin/bash

apt-get update && apt-get install --no-install-recommends -y wget jq python3-rosinstall python3-colcon-common-extensions
pip3 install awscli --upgrade

