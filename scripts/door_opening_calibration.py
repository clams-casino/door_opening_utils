#!/usr/bin/python3

import rospy
import tf
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import tf2_ros
import geometry_msgs.msg


def lookupLatestTransform(listener, target_frame, source_frame):
    try:
        # Wait a bit since after starting this node it could be a bit until the frames will be seen to exist
        listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))

        trans, rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        raise Exception("Could not get the latest TF from {} to {}".format(source_frame, target_frame))
    return trans, rot


def generateTransformStampedMsg(trans, quat, target_frame, source_frame):
    transformStamped = geometry_msgs.msg.TransformStamped()
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = target_frame
    transformStamped.child_frame_id = source_frame

    transformStamped.transform.translation.x = trans[0]
    transformStamped.transform.translation.y = trans[1]
    transformStamped.transform.translation.z = trans[2]

    transformStamped.transform.rotation.x = quat[0]
    transformStamped.transform.rotation.y = quat[1]
    transformStamped.transform.rotation.z = quat[2]
    transformStamped.transform.rotation.w = quat[3]

    return transformStamped


if __name__ == "__main__":
    rospy.init_node('door_opening_calibration')
    tf_listener = tf.TransformListener()


    """ TF from door base to camera frame """
    door_base_to_cam_trans, door_base_to_cam_quat = lookupLatestTransform(tf_listener, "camera_link", "door_base")

    print("\n\n")
    print("{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} camera_link door_base 10\n\n".format(
        door_base_to_cam_trans[0], door_base_to_cam_trans[1], door_base_to_cam_trans[2],
        door_base_to_cam_quat[0], door_base_to_cam_quat[1], door_base_to_cam_quat[2], door_base_to_cam_quat[3]))


    """ TF from camera to the odom frame """
    cam_to_tag_trans, cam_to_tag_quat = lookupLatestTransform(tf_listener, "tag_0_test", "camera_link")
    camera_to_tag = quaternion_matrix(cam_to_tag_quat)
    camera_to_tag[:3, 3] = cam_to_tag_trans

    tag_to_odom_trans, tag_to_odom_quat = lookupLatestTransform(tf_listener, "odom", "robot_tag_0_test")
    tag_to_odom = quaternion_matrix(tag_to_odom_quat)
    tag_to_odom[:3, 3] = tag_to_odom_trans

    camera_to_odom = tag_to_odom @ camera_to_tag
    camera_to_odom_quat = quaternion_from_matrix(camera_to_odom)
    camera_to_odom_trans = camera_to_odom[:3, 3]

    print("\n\n")
    print("{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} odom camera_link 10\n\n".format(
        camera_to_odom_trans[0], camera_to_odom_trans[1], camera_to_odom_trans[2],
        camera_to_odom_quat[0], camera_to_odom_quat[1], camera_to_odom_quat[2], camera_to_odom_quat[3]))


    """ Publish these static transforms """
    print("\n\nPublishing static TFs")
    static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    door_base_to_cam_transform_stamped = generateTransformStampedMsg(
        door_base_to_cam_trans, door_base_to_cam_quat,
        "camera_link", "door_base")

    camera_to_odom_transform_stamped = generateTransformStampedMsg(
        camera_to_odom_trans, camera_to_odom_quat,
        "odom", "camera_link")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        static_tf_broadcaster.sendTransform(door_base_to_cam_transform_stamped)
        static_tf_broadcaster.sendTransform(camera_to_odom_transform_stamped)
        rate.sleep()