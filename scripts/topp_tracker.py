#!/usr/bin/env python

import math
import rospy
import tf
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped, PoseArray
from nav_msgs.msg import Path
from std_srvs.srv import Empty, SetBool
from std_srvs.srv import EmptyResponse, SetBoolResponse
from math import pi, ceil, floor, sqrt
import numpy as np


class TrackerParameters:
    def __init__(self):
        self.from_odom = False
        self.request_permission = True
        self.velocity = [5, 5, 5, 2.5]
        self.acceleration = [2.75, 2.75, 2.75, 1.5]
        self.sampling_frequency = 100
        self.n_gridpoints = 500

class TrackerStatus:
    off = "OFF"
    accept = "ACCEPT"
    wait = "WAIT"
    active = "ACTIVE"

class ToppTracker:

    def __init__(self):
        # Load tracker parameters
        self.tracker_params = TrackerParameters()

        self.tracker_params.n_gridpoints = rospy.get_param("~topp_tracker/n_gridpoints")
        self.tracker_params.sampling_frequency = rospy.get_param("~topp_tracker/sampling_frequency")
        self.tracker_params.request_permission = rospy.get_param("~topp_tracker/request_permission")
        self.tracker_params.from_odom  = rospy.get_param("~topp_tracker/from_odom")
        
        self.rate = 1.0 / self.tracker_params.sampling_frequency
        
        self.tracker_params.velocity[0] = rospy.get_param("~topp_tracker/constraints/velocity/x")
        self.tracker_params.velocity[1] = rospy.get_param("~topp_tracker/constraints/velocity/y")
        self.tracker_params.velocity[2] = rospy.get_param("~topp_tracker/constraints/velocity/z")
        self.tracker_params.velocity[3] = rospy.get_param("~topp_tracker/constraints/velocity/yaw")

        self.tracker_params.acceleration[0] = rospy.get_param("~topp_tracker/constraints/acceleration/x")
        self.tracker_params.acceleration[1] = rospy.get_param("~topp_tracker/constraints/acceleration/y")
        self.tracker_params.acceleration[2] = rospy.get_param("~topp_tracker/constraints/acceleration/z")
        self.tracker_params.acceleration[3] = rospy.get_param("~topp_tracker/constraints/acceleration/yaw")

        self.point_index = 0
        self.trajectory = MultiDOFJointTrajectory()
        self.trajectory_pose_arr = PoseArray()
        self.trajectory_index = 0
        self.traj_sub = rospy.Subscriber("tracker/input_trajectory", MultiDOFJointTrajectory, self.trajectory_cb)
        self.pose_sub = rospy.Subscriber("tracker/input_pose", PoseStamped, self.pose_cb)

        self.carrot_status = String()
        self.carrot_status.data = "HOLD"
        self.status_sub = rospy.Subscriber("carrot/status", String, self.status_cb)
        
        self.carrot_trajectory = MultiDOFJointTrajectoryPoint()
        self.carrot_trajectory_recieved = False
        self.carrot_trajectory_sub = rospy.Subscriber("carrot/trajectory", MultiDOFJointTrajectoryPoint, self.carrot_trajectory_cb)
        self.carrot_pose_sub = rospy.Subscriber("carrot/pose", PoseStamped, self.carrot_pose_cb)
        
        self.odom_msg = Odometry()
        self.odom_recieved = False
        self.odom_sub = rospy.Subscriber("odometry_topic", Odometry, self.odom_cb)

        self.point_pub = rospy.Publisher("output/point", MultiDOFJointTrajectoryPoint, queue_size=1)
        self.pose_pub = rospy.Publisher("output/pose", PoseStamped, queue_size=1)
        self.activity_pub = rospy.Publisher("tracker/status", String, queue_size=1)
        self.path_pub = rospy.Publisher("tracker/path", Path, queue_size=1)
        self.trajectory_pose_arr_pub = rospy.Publisher("tracker/remaining_trajectory", PoseArray, queue_size=1)

        self.enable_trajectory = False
        self.enable_service = rospy.Service("tracker/enable", SetBool, self.enable_service_cb)
        self.topp_reset = rospy.Service("tracker/reset", Empty, self.reset_service_cb)
        
    def odom_cb(self, msg):
        if not self.tracker_params.from_odom:
            rospy.logfatal_throttle(3.0, "ToppTracker - Odometry recieved but not planning from odom")
            return
        
        rospy.loginfo_throttle(3.0, "ToppTracker - recieved odometry - acting as carrot/trajectory")
        transform = Transform()
        transform.translation.x = msg.pose.pose.position.x
        transform.translation.y = msg.pose.pose.position.y
        transform.translation.z = msg.pose.pose.position.z
        transform.rotation = msg.pose.pose.orientation

        self.carrot_trajectory = MultiDOFJointTrajectoryPoint()
        self.carrot_trajectory.transforms.append(transform)
        self.odom_recieved = True


    def reset_service_cb(self, req):
        print("ToppTracker - RESET request recieved")
        self.trajectory = MultiDOFJointTrajectory()
        return EmptyResponse()

    def status_cb(self, msg):
        self.carrot_status = msg
        if not self.carrot_status.data == "HOLD" and self.trajectory.points:
            print("ToppTracker - hold disabled, clearing trajectory!")
            self.trajectory.points = []
            self.trajectory_pose_arr.poses = []
            self.trajectory_index = 0

    def carrot_pose_cb(self, msg):
        if self.tracker_params.from_odom:
            rospy.logwarn_throttle(3.0, "ToppTracker - Carrot trajectory recieved but planning from odom")
            return

        transform = Transform()
        transform.translation.x = msg.pose.position.x
        transform.translation.y = msg.pose.position.y
        transform.translation.z = msg.pose.position.z
        transform.rotation = msg.pose.orientation

        self.carrot_trajectory = MultiDOFJointTrajectoryPoint()
        self.carrot_trajectory.transforms.append(transform)
        self.carrot_trajectory_recieved = True

    def carrot_trajectory_cb(self, msg):
        if self.tracker_params.from_odom:
            rospy.logwarn_throttle(3.0, "ToppTracker - Carrot trajectory recieved but planning from odom")
            return

        self.carrot_trajectory = msg
        self.carrot_trajectory_recieved = True

    def publish_tracker_status(self, status):
        msg = String()
        msg.data = status
        self.activity_pub.publish(msg)

    def enable_service_cb(self, req):
        self.enable_trajectory = req.data
        resp = SetBoolResponse()
        resp.success = True

        if req.data:
            resp.message = "Topp Tracker enabled successfully."
        else:
            resp.message = "Topp Tracker disabled successfully."
            
        return resp
        
    def fix_topp_yaw(self, curr_yaw, previous_yaw):
        delta = previous_yaw - curr_yaw
        if delta > pi:
            return curr_yaw + ceil(floor(abs(delta)/pi)/2.0) * 2 * pi
        elif delta < -pi:
            return curr_yaw - ceil(floor(abs(delta)/pi)/2.0) * 2 * pi
        return curr_yaw

    def interpolate_points(self, start_p, end_p, res=1.0):
        print("Interpolating points")
        x = []
        y = []
        z = []
        yaw = []
        distance = sqrt((end_p.transforms[0].translation.x - start_p.transforms[0].translation.x) ** 2 + 
            (end_p.transforms[0].translation.y - start_p.transforms[0].translation.y) ** 2 + 
            (end_p.transforms[0].translation.z - start_p.transforms[0].translation.z) ** 2)
        print(distance)
        increments = np.linspace(0, 1, round(distance/res), endpoint=False)

        start_yaw = tf.transformations.euler_from_quaternion(
                [start_p.transforms[0].rotation.x, 
                start_p.transforms[0].rotation.y,
                start_p.transforms[0].rotation.z,
                start_p.transforms[0].rotation.w])[2]
        end_yaw = tf.transformations.euler_from_quaternion(
                [end_p.transforms[0].rotation.x, 
                end_p.transforms[0].rotation.y,
                end_p.transforms[0].rotation.z,
                end_p.transforms[0].rotation.w])[2]
        # Unwrap end yaw to interpolate correctly
        end_yaw = self.fix_topp_yaw(end_yaw, start_yaw)

        print("End: ", end_p.transforms[0].translation, end_yaw)
        print("Start: ", start_p.transforms[0].translation, start_yaw)
        for delta in increments:
            x.append((1 - delta) * start_p.transforms[0].translation.x + delta * end_p.transforms[0].translation.x)
            y.append((1 - delta) * start_p.transforms[0].translation.y + delta * end_p.transforms[0].translation.y)
            z.append((1 - delta) * start_p.transforms[0].translation.z + delta * end_p.transforms[0].translation.z)
            yaw.append((1 - delta) * start_yaw + delta * end_yaw)

            if len(yaw) > 1:
                yaw[-1] = self.fix_topp_yaw(yaw[-1], yaw[-2])

        return x, y, z, yaw

    def pose_cb(self, msg):

        # Fill 
        temp_traj = MultiDOFJointTrajectory()
        temp_traj.header = msg.header
        temp_traj.points = []
        temp_traj.points.append(MultiDOFJointTrajectoryPoint())
        temp_traj.points[0].transforms = []
        temp_traj.points[0].transforms.append(Transform())
        temp_traj.points[0].transforms[0].translation.x = msg.pose.position.x
        temp_traj.points[0].transforms[0].translation.y = msg.pose.position.y
        temp_traj.points[0].transforms[0].translation.z = msg.pose.position.z
        temp_traj.points[0].transforms[0].rotation.x = msg.pose.orientation.x
        temp_traj.points[0].transforms[0].rotation.y = msg.pose.orientation.y
        temp_traj.points[0].transforms[0].rotation.z = msg.pose.orientation.z
        temp_traj.points[0].transforms[0].rotation.w = msg.pose.orientation.w        

        # Call trajectory callback with one point
        self.trajectory_cb(temp_traj)
        

    def trajectory_cb(self, msg):
        if len(msg.points) == 0:
            print("ToppTracker - empty input trajectory recieved, RESET")
            self.trajectory = MultiDOFJointTrajectory()
            return
        
        if not self.carrot_trajectory_recieved and not self.tracker_params.from_odom:
            rospy.logwarn("ToppTracker - trajectory recieved but carrot unavailable")
            self.trajectory = MultiDOFJointTrajectory()
            return

        if not self.odom_recieved and self.tracker_params.from_odom:
            rospy.logwarn("ToppTracker - trajectory recieved but odom unavailabel")
            self.trajectory = MultiDOFJointTrajectory()
            return

        # Nicely interpolate points from current to first
        x, y, z, yaw = self.interpolate_points(self.carrot_trajectory, msg.points[0])
        print(x)
        print(y)
        print(yaw)

        if len(x) == 0:
            x.append(self.carrot_trajectory.transforms[0].translation.x)
            y.append(self.carrot_trajectory.transforms[0].translation.y)
            z.append(self.carrot_trajectory.transforms[0].translation.z)
            yaw.append(tf.transformations.euler_from_quaternion(
                [self.carrot_trajectory.transforms[0].rotation.x, 
                self.carrot_trajectory.transforms[0].rotation.y,
                self.carrot_trajectory.transforms[0].rotation.z,
                self.carrot_trajectory.transforms[0].rotation.w])[2])

        for point in msg.points:
            x.append(point.transforms[0].translation.x)
            y.append(point.transforms[0].translation.y)
            z.append(point.transforms[0].translation.z)
            yaw.append(tf.transformations.euler_from_quaternion(
                [point.transforms[0].rotation.x, 
                point.transforms[0].rotation.y,
                point.transforms[0].rotation.z,
                point.transforms[0].rotation.w])[2])

            # Fix Toppra orientation, at this point atleast two points are in trajectory
            if len(yaw) > 1:           
                yaw[-1] = self.fix_topp_yaw(yaw[-1], yaw[-2])
                
        
        for x_,y_,z_,yaw_ in zip(x, y, z, yaw):
            print("Recieved point: ", x_, y_, z_, yaw_)
        
        request = GenerateTrajectoryRequest()
        
        # Add waypoints in request
        waypoint = JointTrajectoryPoint()
        for i in range(0, len(x)):
            # Positions are defined above
            waypoint.positions = [x[i], y[i], z[i], yaw[i]]
            # Also add constraints for velocity and acceleration. These
            # constraints are added only on the first waypoint since the
            # TOPP-RA reads them only from there.
            if i==0:
                waypoint.velocities = [self.tracker_params.velocity[0], self.tracker_params.velocity[1], self.tracker_params.velocity[2], self.tracker_params.velocity[3]]
                waypoint.accelerations = [self.tracker_params.acceleration[0], self.tracker_params.acceleration[1], self.tracker_params.acceleration[2], self.tracker_params.acceleration[3]]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint))

        request.waypoints.joint_names = ["x", "y", "z", "yaw"]
        request.sampling_frequency = self.tracker_params.sampling_frequency
        request.n_gridpoints = self.tracker_params.n_gridpoints
        request.plot = False

        # Request the trajectory
        request_trajectory_service = rospy.ServiceProxy("generate_toppra_trajectory", GenerateTrajectory)
        response = request_trajectory_service(request)

        # Response will have trajectory and bool variable success. If for some
        # reason the trajectory was not able to be planned or the configuration
        # was incomplete or wrong it will return False.
        print ("ToppTracker: Converting trajectory to multi dof")
        joint_trajectory = response.trajectory

        self.enable_trajectory = False

        # Take the first point in the message, extract its roll / pitch and copy it in the 
        # final trajectory
        angles = tf.transformations.euler_from_quaternion([
            msg.points[0].transforms[0].rotation.x, 
            msg.points[0].transforms[0].rotation.y,
            msg.points[0].transforms[0].rotation.z,
            msg.points[0].transforms[0].rotation.w])
        self.trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory, angles[0], angles[1])

        # Publish the path
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = msg.header.frame_id

        self.trajectory_pose_arr.header.stamp = rospy.Time.now()
        self.trajectory_pose_arr.header.frame_id = msg.header.frame_id
        self.trajectory_pose_arr.poses = []
        self.trajectory_index = 0

        for i, point in enumerate(self.trajectory.points):

            path_point = PoseStamped()
            path_point.header.stamp = rospy.Time.now()
            path_point.header.frame_id = msg.header.frame_id
            path_point.pose.position.x = point.transforms[0].translation.x
            path_point.pose.position.y = point.transforms[0].translation.y
            path_point.pose.position.z = point.transforms[0].translation.z
            path_point.pose.orientation.x = point.transforms[0].rotation.x
            path_point.pose.orientation.y = point.transforms[0].rotation.y
            path_point.pose.orientation.z = point.transforms[0].rotation.z
            path_point.pose.orientation.w = point.transforms[0].rotation.w
            path_msg.poses.append(path_point)

            if i % 10 != 0:
                continue

            self.trajectory_pose_arr.poses.append(path_point.pose)

        self.path_pub.publish(path_msg)
        

    def JointTrajectory2MultiDofTrajectory(self, joint_trajectory, roll = 0, pitch = 0):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]

            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, joint_trajectory.points[i].positions[3])
            temp_transform.rotation.x = quaternion[0]
            temp_transform.rotation.y = quaternion[1]
            temp_transform.rotation.z = quaternion[2]
            temp_transform.rotation.w = quaternion[3]

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)
            temp_point.time_from_start = joint_trajectory.points[i].time_from_start

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory

    def run(self):
        while not rospy.is_shutdown():
            
            if not self.tracker_params.from_odom and not self.carrot_trajectory_recieved:
                rospy.loginfo_throttle(1.0, "ToppTracker - Carrot trajectory unavailable")
                self.publish_tracker_status(TrackerStatus.off)
                self.enable_trajectory = False
                rospy.sleep(self.rate)
                continue

            if self.tracker_params.from_odom and not self.odom_recieved:
                rospy.loginfo_throttle(1.0, "ToppTracker - Odom unavailable")
                self.publish_tracker_status(TrackerStatus.off)
                self.enable_trajectory = False
                rospy.sleep(self.rate)
                continue

            if not self.carrot_status.data == "HOLD":
                rospy.loginfo_throttle(1.0, "ToppTracker - Position hold disabled")
                self.publish_tracker_status(TrackerStatus.off)
                self.enable_trajectory = False
                rospy.sleep(self.rate)
                continue
            
            if not self.trajectory.points:
                rospy.loginfo_throttle(1.0, "ToppTracker - No trajectory available")
                self.publish_tracker_status(TrackerStatus.accept)
                self.enable_trajectory = False
                rospy.sleep(self.rate)
                continue
                
            if self.tracker_params.request_permission and not self.enable_trajectory:
                rospy.loginfo_throttle(1.0, "ToppTracker - Do not have a permission to publish trajectory.")
                self.publish_tracker_status(TrackerStatus.wait)
                rospy.sleep(self.rate)
                continue

            # Publish trajectory point
            tmp_point = MultiDOFJointTrajectoryPoint()
            tmp_point = self.trajectory.points.pop(0)
            self.point_pub.publish(tmp_point)

            # PoseStamped
            tmp_pose = PoseStamped()
            tmp_pose.header.stamp = rospy.Time.now()
            tmp_pose.pose.position.x = tmp_point.transforms[0].translation.x
            tmp_pose.pose.position.y = tmp_point.transforms[0].translation.y
            tmp_pose.pose.position.z = tmp_point.transforms[0].translation.z
            tmp_pose.pose.orientation.x = tmp_point.transforms[0].rotation.x
            tmp_pose.pose.orientation.y = tmp_point.transforms[0].rotation.y
            tmp_pose.pose.orientation.z = tmp_point.transforms[0].rotation.z
            tmp_pose.pose.orientation.w = tmp_point.transforms[0].rotation.w

            self.pose_pub.publish(tmp_pose)
            self.publish_tracker_status(TrackerStatus.active)

            if len(self.trajectory_pose_arr.poses) > 0 and \
                self.trajectory_index % 10 == 0:
                self.trajectory_pose_arr.poses.pop(0)
            
            self.trajectory_pose_arr.header.stamp = rospy.Time.now()
            self.trajectory_pose_arr_pub.publish(self.trajectory_pose_arr)            
            self.trajectory_index = self.trajectory_index + 1
            rospy.sleep(self.rate)

if __name__ == "__main__":
    rospy.init_node("uav_ros_tracker")   
    tracker = ToppTracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        pass
