#!/usr/bin/env python

import rospy

class Segment:

	def __init__(self):

		path = rospkg.RosPack().get_path('rail_segmentation')

		self.service = rospy.Service('segment', )