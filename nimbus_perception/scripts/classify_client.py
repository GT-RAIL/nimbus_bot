#!/usr/bin/env python

import sys
import rospy
from nimbus_perception.srv import *

def classify_client(l,a,b,x,y,z):
	rospy.wait_for_service('classify')
	try:
		classify = rospy.ServiceProxy('classify', Classify)
		respl = classify(l,a,b,x,y,z)
		return respl
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
	return "%s [l a b x y z]"%sys.argv[0]

if __name__ == "__main__":

	# initialize inputs
	if len(sys.argv) == 7:
		l = float(sys.argv[1])
		a = float(sys.argv[2])
		b = float(sys.argv[3])
		x = float(sys.argv[4])
		y = float(sys.argv[5])
		z = float(sys.argv[6])
	else:
		print usage()
		sys.exit(1)

	print "Requesting label for [l=%f a=%f b=%f x=%f y=%f z=%f].."%(l, a, b, x, y, z)
	print classify_client(l, a, b, x, y, z)