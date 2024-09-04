#!/usr/bin/env python
# Stream images through Spot
#
# Usage examples:
#
# rosrun rbd_spot_perception stream_image.py

import time
import argparse
import math
import numpy as np

import rospy

import rbd_spot
import pandas as pd
import sys
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from scipy.ndimage import median_filter

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Point, Quaternion
from bosdyn.client.frame_helpers import get_a_tform_b

far_plane = 6000

def publish_extrinsics(image_client, pubs):
        def euler_from_quaternion(w, x, y, z):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
            return roll_x, pitch_y, yaw_z # in radians

        sources_list = ["hand", "frontright_fisheye", "frontleft_fisheye", "left_fisheye", "right_fisheye", "back_fisheye"]
        source_list = [['hand_depth_in_hand_color_frame', 'hand_color_image'], ['frontright_depth_in_visual_frame','frontright_fisheye_image'],
                       ['frontleft_depth_in_visual_frame', 'frontleft_fisheye_image'],
                       ['left_depth_in_visual_frame', 'left_fisheye_image'],
                       ['right_depth_in_visual_frame', 'right_fisheye_image'],
                       ['back_depth_in_visual_frame','back_fisheye_image']
                      ]

        # sources_list = ["frontright_fisheye", "frontleft_fisheye"]#, "hand"]
        # source_list = [['frontright_depth_in_visual_frame','frontright_fisheye_image'],
        #                ['frontleft_depth_in_visual_frame', 'frontleft_fisheye_image']
        #                # ['hand_depth_in_hand_color_frame', 'hand_color_image']
        #               ]

        # Publish each camera source's extrinsics
        for i, source in enumerate(sources_list):
            # Get the tform for this image source
            image_responses = image_client.get_image_from_sources(source_list[i])
            img_snapshot = image_responses[0].shot.transforms_snapshot
            a = "body"
            # a = "vision" # Ground Plane Estimate frame # Are added to test camera orientation with body frame orientation changes
            if source == "hand":
                body_tform = get_a_tform_b(img_snapshot, a,"hand_color_image_sensor")
            else:
                 body_tform = get_a_tform_b(img_snapshot, a, source)
            
            # Calculate the euler angles
            euler = euler_from_quaternion(body_tform.rotation.w, body_tform.rotation.x, body_tform.rotation.y, body_tform.rotation.z)
            euler = [math.degrees(i) for i in euler]
            position = body_tform.position
            
            # Build the message to send to unity
            unity_euler = Quaternion(euler[1], euler[2], euler[0], 1)
            unity_pos = Point(position.y, position.z, position.x)
            pos = Pose(position=unity_pos, orientation=unity_euler)
            
            pubs[i].publish(pos)

def set_far_plane(data):
    global far_plane
    far_plane = float(data.data)

def main():
    parser = argparse.ArgumentParser("stream image")
    parser.add_argument("-i", "--id", help="unique spot id", type=str, default="")
    parser.add_argument("-s", "--sources", nargs="+", help="image sources; or 'list'")
    parser.add_argument("-q", "--quality", type=int,
                        help="image quality [0-100]", default=75)
    formats = ["UNKNOWN", "JPEG", "RAW", "RLE"]
    parser.add_argument("-f", "--format", type=str, default="RAW",
                        help="format", choices=formats)
    parser.add_argument("-p", "--pub", action="store_true", help="publish as ROS messages")
    parser.add_argument("-t", "--timeout", type=float, help="time to keep streaming")
    args = parser.parse_args()

    conn = rbd_spot.SpotSDKConn(sdk_name="StreamImageClient")
    image_client = rbd_spot.image.create_client(conn)

    sources_result, _used_time = rbd_spot.image.listImageSources(image_client)
    print("ListImageSources took %.3fs" % _used_time)
    sources_df = pd.DataFrame(rbd_spot.image.sources_result_to_dict(sources_result))
    print("Available image sources:")
    print(sources_df, "\n")
    with open('stream_info.txt', 'w') as f:
        f.write(str(sources_result))


    if args.sources is None or len(args.sources) == 0:
        # nothing to do.
        return

    # Check if the sources are valid
    ok, bad_sources = rbd_spot.image.check_sources_valid(args.sources, sources_result)
    if not ok and 'extrinsics' not in args.sources[0]:
        print(f"Invalid source name(s): {bad_sources}")
        return

    # Create publishers, in case we want to publish;
    # maps from source name to a publisher.
    # Determine whether this is a depth or extrinsics publisher
    node_name = f"stream_image{args.id}"
    pub_extrinsics = False
    if "depth" in args.sources[0]:
        node_name = f"stream_depth{args.id}"
        farplane_subscriber = rospy.Subscriber('set_far_plane', Float32, set_far_plane, queue_size = 1) 
    elif "extrinsics" in args.sources[0]:
        pub_names = ['hand', 'frontright_fisheye', 'frontleft_fisheye', 'left_fisheye', 'right_fisheye', 'back_fisheye']
        #pub_names = ['frontright_fisheye', 'frontleft_fisheye']#, 'hand']
        pub_names = [f"/spot{args.id}/{name}_extrinsics" for name in pub_names]
        ext_pubs = [rospy.Publisher(name, Pose, queue_size=10) for name in pub_names]
        pub_extrinsics = True
        node_name = f"stream_extrinsics{args.id}"
    if args.pub:
        rospy.init_node(node_name)
        publishers = rbd_spot.image.ros_create_publishers(args.sources, name_space=f"stream_image{args.id}")

    # We want to stream the image sources, publish as ROS message if necessary
    # First, build requests
    rgb_format = dict(image_pb2.Image.PixelFormat.items()).get("PIXEL_FORMAT_RGB_U8")
    image_format = image_pb2.Image.Format.Value(f"FORMAT_{args.format}")
    image_requests = []
    # Build requests iteratively, setting pixel format
    for source in args.sources:
        pixel_format = rgb_format if "fisheye" in source else None
        image_requests.append(build_image_request(source, quality_percent=args.quality, image_format=image_format, pixel_format=pixel_format))

    #image_requests = rbd_spot.image.build_image_requests(
    #    args.sources, quality=args.quality, fmt=args.format, pixel_format=pixel_format)

    # Stream the image through specified sources
    _start_time = time.time()
    while True:
        try:
            if pub_extrinsics:
                publish_extrinsics(image_client, ext_pubs)
            else:
                result, time_taken = rbd_spot.image.getImage(image_client, image_requests)
                # if "depth" in args.sources[0]:
                #     for i in range(len(result)):
                #         cv_depth = np.frombuffer(result[i].shot.image.data, dtype=np.uint16).astype(float)
                #         # Range Linear operation

                #         #near = np.min(cv_depth) # always 0
                #         near = 0
                #         cv_depth[cv_depth >= far_plane] = 0
                #         #cv_depth = cv_depth + 1

                #         #cv_depth = (far * near) / (cv_depth * (far - near))
                #         cv_depth = cv_depth / far_plane

                #         # Quantize to 8 bits
                #         cv_depth = cv_depth * 255
                #         #cv_depth = median_filter(cv_depth, size=3, output=np.ubyte)

                #         # HACK - set cols value to far plane so bytes can be unpacked
                #         result[i].shot.image.cols = int(far_plane)

                #         # Convert the array to single bytes
                #         cv_depth = cv_depth.astype(np.ubyte)
                #         result[i].shot.image.data = bytes(cv_depth)
                #print(f"FPS: {1/time_taken:.2f}")
                if args.pub:
                    rbd_spot.image.ros_publish_image_result(conn, result, publishers)
                _used_time = time.time() - _start_time
                if args.timeout and _used_time > args.timeout:
                    break
        finally:
            if args.pub and rospy.is_shutdown():
                sys.exit(1)

if __name__ == "__main__":
    main()
 
