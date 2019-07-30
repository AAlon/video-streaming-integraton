#!/bin/bash
ros2 topic echo --full-length --csv $1 $2 >> $3
