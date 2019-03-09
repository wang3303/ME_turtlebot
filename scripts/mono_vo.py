#!/usr/bin/env python
import numpy as np
import rospy
import os
from tf import TransformListener
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
from tf.transformations import euler_from_matrix

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

lk_params = dict(winSize=(21, 21),
		criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def featureTracking(image_ref, image_cur, px_ref):
	kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

	st = st.reshape(st.shape[0])
	kp1 = px_ref[st == 1]
	kp2 = kp2[st == 1]

	return kp1, kp2


class PinholeCamera:
	def __init__(self, width, height, fx, fy, cx, cy,
				k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
		self.width = width
		self.height = height
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.distortion = (abs(k1) > 0.0000001)
		self.d = [k1, k2, p1, p2, k3]


class VisualOdometry:
	def __init__(self, cam):
		self.frame_stage = 0
		self.cam = cam
		self.new_frame = None
		self.last_frame = None
		self.cur_R = None
		self.cur_t = None
		self.px_ref = None
		self.px_cur = None
		self.focal = cam.fx
		self.pp = (cam.cx, cam.cy)
		self.trueX, self.trueY, self.trueZ = 0, 0, 0
		self.detector = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
		self.cameraMatrix = [[cam.fx, 0, cam.cx], [0, cam.fy, cam.cy], [0, 0, 1]]

	def processFirstFrame(self):
		self.px_ref = self.detector.detect(self.new_frame)
		# rospy.loginfo(self.px_cur)
		self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
		self.frame_stage = STAGE_SECOND_FRAME

	def processSecondFrame(self):
		self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		# rospy.loginfo(self.px_cur)
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp,
		                               method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp=self.pp)
		self.frame_stage = STAGE_DEFAULT_FRAME
		self.px_ref = self.px_cur

	def processFrame(self):
		self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp,
		                               method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		rospy.loginfo('translation: {}'.format(list(t)))
		rospy.loginfo('rotation: {}'.format(euler_from_matrix(R)))
		self.cur_t = self.cur_t + self.cur_R.dot(t)
		self.cur_R = R.dot(self.cur_R)
		# rospy.loginfo('pose: {}'.format(euler_from_matrix(self.cur_R)))
		if(self.px_ref.shape[0] < kMinNumFeature):
			self.px_cur = self.detector.detect(self.new_frame)
			self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
		self.px_ref = self.px_cur

	def update(self, img):
		assert(img.ndim == 2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
		self.new_frame = img
		if(self.frame_stage == STAGE_DEFAULT_FRAME):
			self.processFrame()
		elif(self.frame_stage == STAGE_SECOND_FRAME):
			self.processSecondFrame()
		elif(self.frame_stage == STAGE_FIRST_FRAME):
			self.processFirstFrame()
		self.last_frame = self.new_frame


class VOnode:
	def __init__(self):
		rospy.init_node('VOnode', anonymous=True)
		self.bridge = CvBridge()
		# camera and laser parameters that get updated
		self.vo = VisualOdometry(PinholeCamera(0, 0, 0, 0, 1, 1))
		self.cam_wh_update = False
		self.cam_matrix_update = False
		self.img_mono8 = None
		self.img_update = False
		self.tf_listener = TransformListener()
		rospy.Subscriber('/camera/image_raw', Image, self.camera_callback, queue_size=1)
		rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)

	def camera_callback(self, msg):
		""" callback for camera images """

		try:
			self.img_mono8 = self.bridge.imgmsg_to_cv2(msg, "mono8")
			# rospy.loginfo("Update image!")
			self.img_update = True
			if not self.cam_wh_update:
				(img_h, img_w) = self.img_mono8.shape
				self.vo.cam.width = img_w
				self.vo.cam.height = img_h
				self.cam_wh_update = True
		except CvBridgeError as e:
			print(e)

	def camera_info_callback(self, msg):
		if not self.cam_matrix_update:
			K = np.array(msg.K)
			self.vo.cam.cx = K[2]
			self.vo.cam.cy = K[5]
			self.vo.cam.fx = K[0]
			self.vo.cam.fy = K[4]
			self.vo.cameraMatrix = [[K[0], 0, K[2]], [0, K[4], K[5]], [0, 0, 1]]
			self.vo.focal = self.vo.cam.fx
			self.vo.pp = (self.vo.cam.cx, self.vo.cam.cy)
			self.cam_matrix_update = True

	def run(self):
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			if self.img_mono8 is not None and self.img_update\
					and self.cam_wh_update and self.cam_matrix_update:
				self.vo.update(self.img_mono8)
				position = self.vo.cur_t
				if self.vo.frame_stage is STAGE_DEFAULT_FRAME:
					x, y, z = position[0], position[1], position[2]
				else:
					x, y, z = 0., 0., 0.
				# rospy.loginfo("x = {}, y = {}, z = {}".format(x, y, z))
			self.img_update = False
			rate.sleep()
		rospy.spin()

if __name__=='__main__':
	d = VOnode()
	d.run()

