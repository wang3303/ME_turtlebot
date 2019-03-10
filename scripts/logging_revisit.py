#!/usr/bin/env python
import rospy
import os
from me_turtlebot.msg import DetectedObject, DetectedObjectList

PATH_TO_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/coco_labels.txt')


def load_object_labels(filename):
	""" loads the coco object readable name """

	fo = open(filename, 'r')
	lines = fo.readlines()
	fo.close()
	object_labels = {}
	for l in lines:
		object_id = int(l.split(':')[0])
		label = l.split(':')[1][1:].replace('\n', '').replace('-', '_').replace(' ', '_')
		object_labels[object_id] = label

	return object_labels


class LogRevisit(object):
	"""
	1. Record the states of the robot when objects are detected
	2. Receive message from topic /delivery_request and send goal to /nav_goal
	"""

	def __init__(self):
		rospy.init_node('log_revisit', anonymous=True)
		self.goal_pub = rospy.Publisher()
		self.request_sub = rospy.Subscriber()
		self.detected_objects = rospy.Publisher('/detector/objects', DetectedObjectList, queue_size=10)
		self.objects_sub = {}

	def request_cb(self, ):
		# goal = ...

		goal_pub.publish(goal)

	def get_target_state(self, ):
		pass