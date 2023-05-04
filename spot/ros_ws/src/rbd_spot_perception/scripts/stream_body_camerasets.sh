#!/bin/bash
rosrun rbd_spot_perception stream_image.py -s\
       frontleft_fisheye_image\
       frontleft_depth_in_visual_frame\
       frontright_fisheye_image\
       frontright_depth_in_visual_frame\
       left_fisheye_image\
       left_depth_in_visual_frame\
       right_fisheye_image\
       right_depth_in_visual_frame\
       back_fisheye_image\
       back_depth_in_visual_frame\
       hand_color_image\
       hand_depth_in_hand_color_frame\
       --format JPEG --pub

#rosrun rbd_spot_perception stream_image.py -s\
#       frontleft_visual_in_depth_frame\
#       frontleft_depth\
#       frontright_visual_in_depth_frame\
#       frontright_depth\
#       left_visual_in_depth_frame\
#       left_depth\
#       right_visual_in_depth_frame\
#       right_depth\
#       back_visual_in_depth_frame\
#       back_depth\
#       hand_color_image\
#       hand_depth_in_hand_color_frame\
#       --pub
