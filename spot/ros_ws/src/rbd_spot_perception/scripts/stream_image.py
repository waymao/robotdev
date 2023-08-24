#!/usr/bin/env python
# Stream images through Spot
#
# Usage examples:
#
# rosrun rbd_spot_perception stream_image.py

import time
import argparse

import rospy

import rbd_spot
import pandas as pd
import sys
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request

def main():
    parser = argparse.ArgumentParser("stream image")
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
    if not ok:
        print(f"Invalid source name(s): {bad_sources}")
        return

    # Create publishers, in case we want to publish;
    # maps from source name to a publisher.
    node_name = "stream_image"
    if "depth" in args.sources[0]:
        node_name = "stream_depth"
    if args.pub:
        rospy.init_node(node_name)
        publishers = rbd_spot.image.ros_create_publishers(args.sources, name_space="stream_image")

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
    num_iterations = 0
    sum_time_taken = 0.0
    while True:
        try:
            #import pdb
            #pdb.set_trace()
            #print(image_requests)
            result, time_taken = rbd_spot.image.getImage(image_client, image_requests)
            sum_time_taken += time_taken
            num_iterations += 1
            if num_iterations == 300:
                print(node_name, " - time taken: ", time_taken, "  , average time taken: ", (sum_time_taken/num_iterations), " -- ", num_iterations)
            # print(time_taken)
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
 
