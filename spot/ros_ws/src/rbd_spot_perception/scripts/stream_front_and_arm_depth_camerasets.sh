#!/bin/bash
# rosrun rbd_spot_perception stream_image.py -s\
#        frontleft_depth_in_visual_frame\
#        frontright_depth_in_visual_frame\
#        -f RAW -q 10 --pub 

# rosrun rbd_spot_perception stream_image.py -s\
#        frontleft_depth_in_visual_frame\
#        frontright_depth_in_visual_frame\
#        hand_depth_in_hand_color_frame\
#        -f RAW -q 10 --pub 


# rosrun rbd_spot_perception stream_image.py -s\
#        frontleft_depth_in_visual_frame\
#        frontright_depth_in_visual_frame\
#        left_depth_in_visual_frame\
#        right_depth_in_visual_frame\
#        back_depth_in_visual_frame\
#        hand_depth_in_hand_color_frame\
#        -f RAW -q 75 --pub 


if [ -z "$1" ]; then
    # empty input
       rosrun rbd_spot_perception stream_image.py -s\
              frontleft_depth_in_visual_frame\
              frontright_depth_in_visual_frame\
              left_depth_in_visual_frame\
              right_depth_in_visual_frame\
              back_depth_in_visual_frame\
              hand_depth_in_hand_color_frame\
              -f RAW -q 75 --pub 
elif [ "$1" -eq 2 ]; then
    # equal to 2
       rosrun rbd_spot_perception stream_image.py -s\
              frontleft_depth_in_visual_frame\
              frontright_depth_in_visual_frame\
              left_depth_in_visual_frame\
              right_depth_in_visual_frame\
              back_depth_in_visual_frame\
              hand_depth_in_hand_color_frame\
              -f RAW -q 75 --pub -i 2
else
    echo "Unsupported argument. Please provide either no argument or number."
fi