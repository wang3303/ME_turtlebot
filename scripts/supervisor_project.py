#!/usr/bin/env python

import rospy
# from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf, tf.transformations
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
import math
from enum import Enum
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from grids import StochOccupancyGrid2D

# threshold at which we consider the robot at a location
POS_EPS = .3
THETA_EPS = .4

# time to stop at a stop sign
STOP_SIGN_STOP_TIME = 3

# time to stop at an vendor
GOAL_STOP_TIME = 3

# minimum pic area from a stop sign to obey it
STOP_MIN_DIST = 2000

# min pic area it needs to see to detect vendor
VENDOR_MIN_DIST = 5000

# time taken to cross an intersection
CROSSING_TIME = 6

# time taken to cross a detected vendor
FOOD_PICKUP_TIME = 6

# Time to wait for recording origin location
WAIT_FOR_ORIGIN_TIME = 5

# time taken to cross a detected vendor
VENDOR_CROSSING_TIME = 6

FOOD_LIST = ['pizza', 'cake', 'apple']

# nubmer of vendors to visit
NUM_VENDORS_TO_VISIT = len(FOOD_LIST) - 1


# state machine modes, not all implemented
class Mode(Enum):
	IDLE = 1
	POSE = 2
	STOP = 3
	CROSS = 4
	NAV = 5
	MANUAL = 6
	EXP = 7
	VENDOR_CROSS = 8
	WAIT_FOR_ORIGIN = 9
	ADD_GAS = 10
	WAIT_FOR_ORDER = 11


class Supervisor:
	""" the state machine of the turtlebot """

	def __init__(self):
		rospy.init_node('project_supervisor', anonymous=True)

		# timer
		self.stop_start = 0.0
		self.stop_time = 0.0
		self.vendor_cross_start = 0.0

		# current pose
		self.x = 0
		self.y = 0
		self.theta = 0

		# delivery address
		self.x_d = 0
		self.y_d = 0
		self.theta_d = 0

		# pose goal
		self.x_g = 0
		self.y_g = 0
		self.theta_g = 0

		# current mode
		# TODO: remember to change back
		self.mode = Mode.WAIT_FOR_ORIGIN
		# self.mode = Mode.WAIT_FOR_ORDER
		self.last_mode_printed = None
		self.originWaitTime_start = rospy.get_rostime()

		self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
		self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
		self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.wait_for_orders_publisher = rospy.Publisher('/ready_to_pickup', Bool, queue_size=10)
		# visualize robot in rviz
		self.rviz_state_publisher = rospy.Publisher('rviz_turtle_state', Marker, queue_size=5)

		rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
		rospy.Subscriber('/ready_to_pickup', Bool, self.pickup_callback)
		rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

		self.request_sub = rospy.Subscriber('/delivery_request', String, self.request_cb)

		# initializes the first goal as the origin of mag (where the bot first )
		self.goal_name = []
		self.goal_list = []
		self.food_to_get = []

		self.phaseI = True

		# set up object detector
		for food in FOOD_LIST:
			rospy.Subscriber('/detector/' + food, DetectedObject, self.vendor_detected_callback)

		self.trans_listener = tf.TransformListener()
		# -------------------------------------------------------------
		# to calculate best sequence of visiting
		# map parameters
		self.map_width = 0
		self.map_height = 0
		self.map_resolution = 0
		self.map_origin = [0, 0]
		self.map_probs = []
		self.occupancy = None
		self.occupancy_updated = False
		# plan parameters
		self.plan_resolution = 0.04
		self.plan_horizon = 15
		# subscriber to map information
		rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
		rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
		# -------------------------------------------------------------
	# -------------------------------------------------------------
	def map_md_callback(self, msg):
		self.map_width = msg.width
		self.map_height = msg.height
		self.map_resolution = msg.resolution
		self.map_origin = (msg.origin.position.x, msg.origin.position.y)

	def map_callback(self, msg):
		self.map_probs = msg.data
		if self.map_width > 0 and self.map_height > 0 and len(self.map_probs) > 0:
			self.occupancy = StochOccupancyGrid2D(self.map_resolution,
			                                      self.map_width,
			                                      self.map_height,
			                                      self.map_origin[0],
			                                      self.map_origin[1],
			                                      8,
			                                      self.map_probs)
			self.occupancy_updated = True

	# -------------------------------------------------------------
	def rviz_turtlebot_state(self):
		origin_frame = "/map"
		marker = Marker(
			type=Marker.CYLINDER,
			id=0,
			lifetime=rospy.Duration(1.),
			pose=Pose(Point(self.x, self.y, 0.0), Quaternion(0, 0, 0, 1)),
			scale=Vector3(0.2, 0.2, 0.2),
			header=Header(frame_id=origin_frame, stamp=rospy.Time.now()),
			color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
		)
		self.rviz_state_publisher.publish(marker)

	def request_cb(self, msg):
		rospy.loginfo('Receiving request!')
		self.food_to_get = msg.data.split(',')
		self.phaseI = False

	def pickup_callback(self, msg):
		if self.mode == Mode.WAIT_FOR_ORDER:
			if msg.data:
				rospy.loginfo("ORDERS RECEIVED")
				self.mode = Mode.NAV

	def rviz_goal_callback(self, msg):
		""" callback for a pose goal sent through rviz """
		self.x_g = msg.pose.position.x
		self.y_g = msg.pose.position.y
		rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(rotation)
		self.theta_g = euler[2]

	# TODO
	# self.mode = Mode.NAV

	def stop_sign_detected_callback(self, msg):
		""" callback for when the detector has found a stop sign. Note that
		a distance of 0 can mean that the lidar did not pickup the stop sign at all """

		y_min = msg.corners[0]
		y_max = msg.corners[2]
		x_min = msg.corners[1]
		x_max = msg.corners[3]

		boxArea = (x_max - x_min) * (y_max - y_min)
		rospy.loginfo("STOP SIGN AREA: %f", boxArea)

		# if close enough and in nav mode, stop
		if boxArea > STOP_MIN_DIST and (self.mode == Mode.NAV):
			self.init_stop_sign()

	# -------------------------------------------------------------
	def construct_goal_lists(self, ):
		# name, goal = self.goal_name, self.goal_list
		# for i in range(len())
		#     idx =
		#     name.append(self.goal_list[idx])
		#     goal.append(self.goal_name[idx])
		# TODO: modify the goal list after receiving the request
		name = []
		goal = []
		for i in range(len(self.goal_name)):
			if self.goal_name[i] in self.food_to_get:
				name.append(self.goal_name[i])
				goal.append(self.goal_list[i])

		self.goal_list, self.goal_name = goal, name

		rospy.loginfo("construct_goal_lists: goal_name: {} goal_list: {}".format(self.goal_name, self.goal_list))

		dic = self.construc_dic()
		sorted_name = self.get_visit_sequence('start_point', self.goal_name, dic)
		sorted_goal = []
		for n in sorted_name:
			for i in range(len(name)):
				if name[i] == n:
					sorted_goal.append(goal[i])

		# append start point to the end of the list
		sorted_goal.append((self.x, self.y, self.theta))
		sorted_name.append('start_point')
		rospy.loginfo("construct_goal_lists: sorted_goal: {} sorted_name: {}".format(sorted_goal, sorted_name))
		return sorted_goal, sorted_name
	# -------------------------------------------------------------
	def snap_to_grid(self, x):
		return (self.plan_resolution * round(x[0] / self.plan_resolution),
		        self.plan_resolution * round(x[1] / self.plan_resolution))

	# construc_dic() will return a dictionary. Ex. {('start_point','Apple'):15.00, ('start_point','Banana'):15.00}
	def construc_dic(self, ):
		from astar import AStar
		name = ['start_point']
		goal = [(self.x, self.y, self.theta)]
		name += self.goal_name
		goal += self.goal_list

		dic = {}

		state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))  # not sure what is plan.hozizon
		state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))  # not sure what is plan.hozizon
		rospy.loginfo("Start construction dic")
		for i in range(len(name) - 1):
			for j in range(i + 1, len(name)):
				x_init = self.snap_to_grid((goal[i][0], goal[i][1]))
				x_goal = self.snap_to_grid((goal[j][0], goal[j][1]))
				problem = AStar(state_min, state_max, x_init, x_goal, self.occupancy, self.plan_resolution)
				if not problem.solve():
					return False
				# add into dictionary
				dic[(name[i], name[j])] = problem.calculate_path_lenth()
				dic[(name[j], name[i])] = dic[(name[i], name[j])]

		rospy.loginfo("construc_dic: dic: {}".format(dic))
		rospy.loginfo("dic size {}".format(len(dic)))
		return dic


	def get_visit_sequence(self, init, request_list, distances):
		"""
		:param init: the name of origin point
		:param request_list: the names of the points to visit
		:param distances: a dictionary consisting of (point1, point2): distance(point1, point2)
		:return: the list of sequence of name to visit
		"""

		def helper(visited, unvisited_num, length):
			if unvisited_num == 0:
				length += distances[(visited[-1], init)]
				return length, visited[1:]
			min_len = float('inf')
			sequence = None
			for pos in request_list:
				if pos not in visited:
					res = helper(visited + [pos], unvisited_num - 1, length + distances[(visited[-1], pos)])
					if res[0] < min_len:
						min_len = res[0]
						sequence = res[1]
			return min_len, sequence

		return helper([init], len(request_list), 0)[1]
	# -------------------------------------------------------------

	def vendor_detected_callback(self, msg):
		""" callback for when the detector has found an vendor. Note that
		a distance of 0 can mean that the lidar did not pickup the stop sign at all """

		# distance of the vendor
		y_min = msg.corners[0]
		y_max = msg.corners[2]
		x_min = msg.corners[1]
		x_max = msg.corners[3]

		boxArea = (x_max - x_min) * (y_max - y_min)
		rospy.loginfo("VENDOR AREA: %f", boxArea)
		# need to calibrate this number
		dist = math.sqrt(boxArea) * 0.0025
		dist = 1 / dist
		rospy.loginfo("Predicted dist: %f", dist)

		# if close enough and in nav mode, stop
		if boxArea > VENDOR_MIN_DIST and self.mode == Mode.EXP:
			self.goal_list.append((self.x, self.y, self.theta))
			self.goal_name.append(msg.name)
			rospy.loginfo("vendor_detected_callback: {}".format(msg.name))
			rospy.loginfo("vendor_detected_callback: VENDOR LIST: {}".format(self.goal_list))
			self.init_vendor_cross()

	def init_vendor_cross(self):
		""" initiates a vendor crossing maneuver """
		self.vendor_cross_start = rospy.get_rostime()
		self.mode = Mode.VENDOR_CROSS

	def has_crossed_vendor(self):
		""" initiates an intersection crossing maneuver """
		return (self.mode == Mode.VENDOR_CROSS and
		        (rospy.get_rostime() - self.vendor_cross_start) >
		        rospy.Duration.from_sec(VENDOR_CROSSING_TIME))

	def go_to_pose(self):
		""" sends the current desired pose to the pose controller """
		pose_g_msg = Pose2D()
		pose_g_msg.x = self.x_g
		pose_g_msg.y = self.y_g
		pose_g_msg.theta = self.theta_g

		self.pose_goal_publisher.publish(pose_g_msg)

	def nav_to_pose(self):
		""" sends the current desired pose to the naviagtor """

		nav_g_msg = Pose2D()
		nav_g_msg.x = self.x_g
		nav_g_msg.y = self.y_g
		nav_g_msg.theta = self.theta_g

		self.nav_goal_publisher.publish(nav_g_msg)

	def stay_idle(self):
		""" sends zero velocity to stay put """

		vel_g_msg = Twist()
		self.cmd_vel_publisher.publish(vel_g_msg)

	def close_to(self, x, y, theta):
		""" checks if the robot is at a pose within some threshold """

		return (abs(x - self.x) < POS_EPS and abs(y - self.y) < POS_EPS and abs(theta - self.theta) < THETA_EPS)

	def init_stop_sign(self):
		""" initiates a stop sign maneuver """
		rospy.loginfo("STOP SIGN INIT MANEUVER")
		self.stop_start = rospy.get_rostime()
		self.stop_time = STOP_SIGN_STOP_TIME
		self.mode = Mode.STOP

	def init_goal(self):
		""" initiates a vendor maneuver """

		self.stop_start = rospy.get_rostime()
		self.stop_time = GOAL_STOP_TIME
		self.mode = Mode.STOP

	def has_stopped(self):
		""" checks if stop sign maneuver is over """

		return (self.mode == Mode.STOP and (rospy.get_rostime() - self.stop_start) > rospy.Duration.from_sec(
			self.stop_time))

	def init_crossing(self):
		""" initiates an intersection crossing maneuver """

		self.cross_start = rospy.get_rostime()
		self.mode = Mode.CROSS

	def has_crossed(self):
		""" checks if crossing maneuver is over """

		return (self.mode == Mode.CROSS and (rospy.get_rostime() - self.cross_start) > rospy.Duration.from_sec(
			CROSSING_TIME))

	def loop(self):
		""" the main loop of the robot. At each iteration, depending on its
		mode (i.e. the finite state machine's state), if takes appropriate
		actions. This function shouldn't return anything """

		try:
			(translation, rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			self.x = translation[0]
			self.y = translation[1]
			euler = tf.transformations.euler_from_quaternion(rotation)
			self.theta = euler[2]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

		assert len(self.goal_name) == len(self.goal_list)

		# logs the current mode
		if not (self.last_mode_printed == self.mode):
			rospy.loginfo("Current Mode: %s", self.mode)
			self.last_mode_printed = self.mode

		# checks wich mode it is in and acts accordingly
		if self.mode == Mode.WAIT_FOR_ORIGIN:
			self.stay_idle()

			if (rospy.get_rostime() - self.originWaitTime_start) > rospy.Duration.from_sec(WAIT_FOR_ORIGIN_TIME):
				self.mode = Mode.EXP

		elif self.mode == Mode.IDLE:
			# send zero velocity
			self.stay_idle()

		elif self.mode == Mode.POSE:
			# moving towards a desired pose
			if self.close_to(self.x_g, self.y_g, self.theta_g):
				self.mode = Mode.IDLE
			else:
				self.go_to_pose()

		elif self.mode == Mode.STOP:
			# at a stop sign
			self.stay_idle()

			if self.has_stopped():
				self.init_crossing()
			else:
				pass
		elif self.mode == Mode.VENDOR_CROSS:
			if self.has_crossed_vendor():
				self.mode = Mode.EXP
			else:
				pass

		elif self.mode == Mode.CROSS:
			# crossing an intersection
			if self.has_crossed():
				self.mode = Mode.NAV
			else:
				self.nav_to_pose()

		elif self.mode == Mode.NAV:
			# if in phase ii=
			if not self.phaseI:
				if len(self.goal_list) != 0:
					self.x_g = self.goal_list[0][0]
					self.y_g = self.goal_list[0][1]
					self.theta_g = self.goal_list[0][2]
					rospy.loginfo("Mode.NAV: continue, GOAL_List:{}".format(len(self.goal_list)))
				if self.close_to(self.x_g, self.y_g, self.theta_g) and len(self.goal_list) == 0:
					rospy.loginfo('Mode.NAV: Finish delivery!')
					self.mode = Mode.IDLE

				elif self.close_to(self.x_g, self.y_g, self.theta_g) and len(self.goal_list) != 0:
					rospy.loginfo("Mode.NAV: arrived at {}".format(self.goal_name[0]))
					rospy.loginfo("Mode.NAV: MOVE ON TO Next point")
					del self.goal_list[0]
					del self.goal_name[0]
					self.mode = Mode.STOP
					self.init_goal()
				else:
					self.nav_to_pose()
			# if in phase I
			else:
				# if self.close_to(self.x_g, self.y_g, self.theta_g):
				#     self.mode = Mode.IDLE
				# else:
				#     self.nav_to_pose()
				if not self.close_to(self.x_g, self.y_g, self.theta_g):
					self.nav_to_pose()
				self.mode = Mode.EXP

		elif self.mode == Mode.EXP:
			if len(self.goal_list) == 0:
				# adding in the location of the delivery address
				self.origin_loc = (self.x, self.y, self.theta)
				rospy.loginfo("Origin in EXP: {}".format(self.origin_loc))
				self.goal_list.append(self.origin_loc)
				self.goal_name.append('origin')
				rospy.loginfo("START EXPLORING THE TOWN")
			elif len(self.goal_list) >= NUM_VENDORS_TO_VISIT + 1:
				rospy.loginfo("ALL VENDORS LOCATED")
				self.x_g = self.goal_list[0][0]
				self.y_g = self.goal_list[0][1]
				self.theta_g = self.goal_list[0][2]
				del self.goal_list[0]
				del self.goal_name[0]
				self.goal_list.append(self.origin_loc)
				self.goal_name.append('origin')
				self.mode = Mode.ADD_GAS
			else:
				self.nav_to_pose()

		elif self.mode == Mode.ADD_GAS:
			if self.close_to(self.x_g, self.y_g, self.theta_g):
				rospy.loginfo('Mode.ADD_GAS: Arrive at home!')
				# self.x_g = self.goal_list[0][0]
				# self.y_g = self.goal_list[0][1]
				# self.theta_g = self.goal_list[0][2]
				# del self.goal_list[0]
				# del self.goal_name[0]
				rospy.loginfo('Mode.ADD_GAS -> WAIT_FOR_ORDER')
				# print(self.goal_name)
				# rospy.loginfo('111111111111111111111111111111111111111111111')
				self.mode = Mode.WAIT_FOR_ORDER
			else:
				rospy.loginfo('Mode.ADD_GAS: On the way back to home!')
				self.nav_to_pose()

		elif self.mode == Mode.WAIT_FOR_ORDER:

			if len(self.food_to_get) is 0:
				rospy.loginfo("WAITING FOR ORDERS...")
				print(self.food_to_get)
			else:
				try:
					goal, name = self.construct_goal_lists()
				except:
					rospy.loginfo("CANNOT CALCULATE PATH")
					name = []
					goal = []
					for i in range(len(self.goal_name)):
						if self.goal_name[i] in self.food_to_get:
							name.append(self.goal_name[i])
							goal.append(self.goal_list[i])
					goal.append((self.x, self.y, self.theta))
					name.append('start_point')
				self.goal_list, self.goal_name = goal, name
				self.wait_for_orders_publisher.publish(True)
				# self.phaseI = False
		else:
			raise Exception('This mode is not supported: %s'
			                % str(self.mode))

	def run(self):
		rate = rospy.Rate(10)  # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()


if __name__ == '__main__':
	sup = Supervisor()
	sup.run()
