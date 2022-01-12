#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from uav_ros_msgs.msg import Waypoint, Waypoints
from std_srvs.srv import Empty, EmptyResponse

class RvizClicker:

    def __init__(self):
    
        # Parameters
        self.WaypointHeight = 1.0
        self.WaitingTime = 10.0
        self.WaypointYaw = 0
        self.WaypointFrame = "eagle/carto_raw"
       
        # Waypoints array
        self.waypoints_msg = Waypoints()

        # Setup subscribers
        self.rviz_point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb)
        self.rviz_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.clicked_pose_cb)
        # Setup publishers
        self.wp_array_pub = rospy.Publisher("rviz_clicker/waypoint_array", PoseArray, queue_size=1)
        self.waypoints_pub = rospy.Publisher("waypoints", Waypoints, queue_size=1)
        
        # Setup timers
        self.wp_array_timer = rospy.Timer(rospy.Duration(0.1), self.wp_array_loop)

        # Setup services
        rospy.Service("rviz_clicker/clear_waypoints", Empty, self.clear_waypoints_srv)         
        rospy.Service("rviz_clicker/delete_last", Empty, self.delete_last_wp_srv)
        rospy.Service("rviz_clicker/publish_mission", Empty, self.publish_mission_srv)
   
    def publish_mission_srv(self, msg):
        if self.waypoints_msg.waypoints:
            self.waypoints_pub.publish(self.waypoints_msg)
            self.waypoints_msg.waypoints.clear()
        return EmptyResponse()

    def delete_last_wp_srv(self, msg):
        if self.waypoints_msg.waypoints:
            self.waypoints_msg.waypoints.pop()
        return EmptyResponse()

    def clear_waypoints_srv(self, msg):
        self.waypoints_msg.waypoints.clear()
        resp = EmptyResponse()
        return EmptyResponse()

    def wp_array_loop(self, event):
        wp_array = PoseArray()
        wp_array.header.frame_id = self.WaypointFrame
        wp_array.header.stamp = rospy.Time.now()
        for waypoint in self.waypoints_msg.waypoints:
            wp_array.poses.append(waypoint.pose.pose)
            wp_array.header.frame_id = waypoint.pose.header.frame_id
            wp_array.header.stamp = rospy.Time.now()
        self.wp_array_pub.publish(wp_array)

    def clicked_pose_cb(self, msg):
        waypoint = Waypoint()
        waypoint.waiting_time = self.WaitingTime
        waypoint.pose.header.frame_id = msg.header.frame_id
        waypoint.pose.header.stamp = rospy.Time.now()

        # Set position
        waypoint.pose.pose.position.x = msg.pose.position.x
        waypoint.pose.pose.position.y = msg.pose.position.y
        waypoint.pose.pose.position.z = self.WaypointHeight
        
        # Set orientation
        waypoint.pose.pose.orientation.x = msg.pose.orientation.x
        waypoint.pose.pose.orientation.y = msg.pose.orientation.y
        waypoint.pose.pose.orientation.z = msg.pose.orientation.z
        waypoint.pose.pose.orientation.w = msg.pose.orientation.w

        # Append
        self.waypoints_msg.waypoints.append(waypoint)
        print("Got waypoint: \n", waypoint)


    def clicked_point_cb(self, msg):
        waypoint = Waypoint()
        waypoint.waiting_time = self.WaitingTime
        waypoint.pose.header.frame_id = self.WaypointFrame
        waypoint.pose.header.stamp = rospy.Time.now()
    
        # Set position
        waypoint.pose.pose.position.x = msg.point.x
        waypoint.pose.pose.position.y = msg.point.y
        waypoint.pose.pose.position.z = self.WaypointHeight
        
        # Set orientation
        wp_quaternion = quaternion_from_euler(0, 0, self.WaypointYaw)
        waypoint.pose.pose.orientation.x = wp_quaternion[0]
        waypoint.pose.pose.orientation.y = wp_quaternion[1]
        waypoint.pose.pose.orientation.z = wp_quaternion[2]
        waypoint.pose.pose.orientation.w = wp_quaternion[3]

        # Append
        self.waypoints_msg.waypoints.append(waypoint)
        print("Got waypoint: \n", waypoint)


if __name__ == '__main__':
    rospy.init_node("rviz_clicker")
    rviz_clicker = RvizClicker()
    rospy.spin()