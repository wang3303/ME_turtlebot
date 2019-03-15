#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseArray, Pose2D
import tf, tf.transformations
import numpy as np
from numpy import linalg
from utils import wrapToPi, wrapTo2Pi
from enum import Enum

# gains
Kv = 0.5
Kw = 0.5

TIMEOUT = np.inf

V_MAX = 0.1

# maximim angular velocity
W_MAX = 0.5

DIST_THRES = 0.06
YAW_THRES = np.pi * (10 / 180.0)

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


class ControllerState(Enum):
	IDLE = 1
	FWD = 2
	FIX_YAW = 3


class PoseController:
	def __init__(self):
		rospy.init_node('turtlebot_pose_controller_nav', log_level=rospy.INFO, anonymous=True)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.x_g = None
		self.y_g = None
		self.theta_g = None
		self.state = ControllerState.IDLE
		# time last pose command was received
		self.cmd_pose_time = rospy.get_rostime()
		self.trans_listener = tf.TransformListener()

		rospy.Subscriber('/cmd_pose', Pose2D, self.cmd_pose_callback)

	def update_state(self, state):
		if state != self.state:
			rospy.loginfo("PoseController: State changed to {}".format(state))
			self.state = state

	def cmd_pose_callback(self, data):
		rospy.logdebug("in cmd pose callback")
		if data.x == self.x_g and data.y == self.y_g:
			return

		self.x_g = data.x
		self.y_g = data.y
		self.theta_g = data.theta

		self.cmd_pose_time = rospy.get_rostime()
		rospy.loginfo("PoseController: Got new goal x:{} y:{}, theta:{}".format(self.x_g, self.y_g, self.theta_g))
		# start moving, always turn first
		self.update_state(ControllerState.FWD)

	def update_current_pose(self):
		if not use_gazebo:
			try:
				origin_frame = "/map" if mapping else "/odom"
				(translation, rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint',
				                                                              rospy.Time(0))
				self.x = translation[0]
				self.y = translation[1]
				euler = tf.transformations.euler_from_quaternion(rotation)
				self.theta = euler[2]
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				pass

	def get_ctrl_output_fwd(self):
		if (rospy.get_rostime().to_sec() - self.cmd_pose_time.to_sec()) < TIMEOUT:
			rel_coords = np.array([self.x - self.x_g, self.y - self.y_g])
			R = np.array([[np.cos(self.theta_g), np.sin(self.theta_g)], [-np.sin(self.theta_g), np.cos(self.theta_g)]])
			rho = linalg.norm(rel_coords)
			th_rot = wrapToPi(wrapTo2Pi(self.get_direction(self.x_g, self.y_g) - wrapTo2Pi(self.theta)))

			if np.abs(th_rot) < YAW_THRES:
				V = Kv * rho
				om = 0
			else:
				V = 0
				om = Kw * th_rot
			cmd_x_dot = np.sign(V) * min(V_MAX, np.abs(V))
			cmd_theta_dot = np.sign(om) * min(W_MAX, np.abs(om))
		else:
			cmd_x_dot = 0
			cmd_theta_dot = 0

		cmd = Twist()
		cmd.linear.x = cmd_x_dot
		cmd.angular.z = cmd_theta_dot
		return cmd

	def get_direction(self, x, y):
		return np.arctan2(y - self.y, x - self.x)

	def get_ctrl_output_fix_yaw(self, theta_target):
		cmd_x_dot = 0
		cmd_theta_dot = 0

		if (rospy.get_rostime().to_sec() - self.cmd_pose_time.to_sec()) < TIMEOUT:
			err_yaw = wrapToPi(wrapTo2Pi(theta_target) - wrapTo2Pi(self.theta))
			if np.abs(err_yaw) > YAW_THRES:
				rospy.logdebug("yaw error: {}".format(err_yaw))
				cmd_theta_dot = Kw * err_yaw

		else:
			cmd_x_dot = 0
			cmd_theta_dot = 0
			err_yaw = 0

		cmd = Twist()
		cmd.linear.x = cmd_x_dot
		cmd.angular.z = cmd_theta_dot
		return cmd, err_yaw

	def close_to_goal(self):
		self.update_current_pose()
		rel_coords = np.array([self.x - self.x_g, self.y - self.y_g])
		rho = linalg.norm(rel_coords)
		return rho < DIST_THRES

	def get_ctrl_output_idle(self):
		cmd = Twist()
		cmd.linear.x = 0
		cmd.angular.z = 0
		return cmd

	def run(self):
		rate = rospy.Rate(10)  # 10 Hz
		while not rospy.is_shutdown():
			if self.x_g is None:
				self.update_state(ControllerState.IDLE)
			# State machine for pose controller
			if self.state == ControllerState.IDLE:
				ctrl_output = self.get_ctrl_output_idle()
			elif self.state == ControllerState.FWD:
				self.update_current_pose()
				ctrl_output = self.get_ctrl_output_fwd()
				if self.close_to_goal():
					self.update_state(ControllerState.FIX_YAW)
			elif self.state == ControllerState.FIX_YAW:
				self.update_current_pose()
				ctrl_output, err_yaw = self.get_ctrl_output_fix_yaw(self.theta_g)
				if abs(err_yaw) < YAW_THRES:
					self.update_state(ControllerState.IDLE)

			if self.state is not ControllerState.IDLE:
				rospy.loginfo('close_to_goal Publish from pose_controller: {}, state:{}'.format(ctrl_output, self.state))
				self.pub.publish(ctrl_output)
			rate.sleep()


if __name__ == '__main__':
	posecontrol = PoseController()
	posecontrol.run()
