#!/bin/bash
# rosrun rbd_spot_perception stream_image.py -s\
#        frontleft_fisheye_image\
#        frontright_fisheye_image\
#        left_fisheye_image\
#        right_fisheye_image\
#        back_fisheye_image\
#        -f JPEG -q 50 --pub


# rosrun rbd_spot_perception stream_image.py -s\
#        frontleft_fisheye_image\
#        frontright_fisheye_image\
#        left_fisheye_image\
#        right_fisheye_image\
#        back_fisheye_image\
#        hand_color_image\
#        -f JPEG -q 50 --pub

if [ -z "$1" ]; then
    # empty input
    rosrun rbd_spot_perception stream_image.py -s \
       frontleft_fisheye_image \
       frontright_fisheye_image \
       left_fisheye_image \
       right_fisheye_image \
       back_fisheye_image \
       hand_color_image \
       -f JPEG -q 50 --pub
elif [ "$1" -eq 2 ]; then
    # equal to 2
    rosrun rbd_spot_perception stream_image.py -s \
       frontleft_fisheye_image \
       frontright_fisheye_image \
       left_fisheye_image \
       right_fisheye_image \
       back_fisheye_image \
       hand_color_image \
       -f JPEG -q 50 --pub -i 2
else
    echo "Unsupported argument. Please provide either no argument or number."
fi