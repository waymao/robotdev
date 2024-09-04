#!/bin/bash
#rosrun rbd_spot_perception stream_image.py -s extrinsics -f JPEG -q 50 --pub

if [ -z "$1" ]; then
    # empty input
       rosrun rbd_spot_perception stream_image.py -s extrinsics -f JPEG -q 50 --pub
elif [ "$1" -eq 2 ]; then
    # equal to 2
       rosrun rbd_spot_perception stream_image.py -s extrinsics -f JPEG -q 50 --pub -i 2
else
    echo "Unsupported argument. Please provide either no argument or number."
fi