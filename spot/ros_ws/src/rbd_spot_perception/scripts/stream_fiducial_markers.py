# Copyright (c) 2021 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Example using the world objects service. """

from __future__ import print_function
import argparse
import sys
import time
import roslib
import rospy

import tf

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.api import world_object_pb2


def main(argv):
    """An example using the API to list and get specific objects."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    options = parser.parse_args(argv)

    # Create robot object with a world object client.
    sdk = bosdyn.client.create_standard_sdk('WorldObjectClient')
    robot = sdk.create_robot(options.hostname)
    robot.authenticate(options.username, options.password)
    # Time sync is necessary so that time-based filter requests can be converted.
    robot.time_sync.wait_for_sync()

    # Create the world object client.
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

    br = tf.TransformBroadcaster()

    fiducial_poses = {}

    while not rospy.is_shutdown():

        
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = world_object_client.list_world_objects(
        object_type=request_fiducials).world_objects

        fiducials = []

        for fiducial in fiducial_objects:
            fiducial_number = fiducial.name.split("_")[-1]
            fiducial_name = "fiducial_"+str(fiducial_number)

            fiducials.append(fiducial_name)

            fiducial_transform = fiducial.transforms_snapshot.child_to_parent_edge_map[fiducial_name]

            parent_frame_name = fiducial_transform.parent_frame_name
            fiducial_position = fiducial_transform.parent_tform_child.position
            fiducial_rotation = fiducial_transform.parent_tform_child.rotation

            if "fisheye" not in parent_frame_name:
                fiducial_poses[fiducial_name] = [fiducial_position,fiducial_rotation,parent_frame_name]

        for fiducial_key, fiducial_value in fiducial_poses.items():

            if "fisheye" not in fiducial_value[2]:
                br.sendTransform((fiducial_value[0].x, fiducial_value[0].y,fiducial_value[0].z),
                                 (fiducial_value[1].x,fiducial_value[1].y,fiducial_value[1].z,fiducial_value[1].w),
                                 rospy.Time.now(),
                                 fiducial_key,
                                 fiducial_value[2])

                print(fiducial_key)
                print(fiducial_value[2])
                print(fiducial_value[0], fiducial_value[1])
        print("========================")




if __name__ == '__main__':
    rospy.init_node('spot_fiducial_marker_broadcaster')
    if not main(sys.argv[1:]):
        sys.exit(1)