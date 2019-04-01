#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def main():
    rospy.init_node('my_node')
    rate = rospy.Rate(1);
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    shape = Marker.CUBE

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "/map";
        marker.header.stamp = rospy.Time.now();
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = shape
        marker.action = Marker.ADD
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rospy.Duration()
        marker_publisher.publish(marker)

        if shape == Marker.CUBE:
            shape = Marker.SPHERE
        elif shape == Marker.SPHERE:
            shape = Marker.ARROW
        elif shape == Marker.ARROW:
            shape = Marker.CYLINDER
        else:
            shape = Marker.CUBE

        rate.sleep()

if __name__ == '__main__':
    main()
