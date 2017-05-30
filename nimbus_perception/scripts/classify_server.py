#!/usr/bin/env python

import rospy
import rospkg
import pandas as pandas
from nimbus_perception.srv import Classify
from sklearn.externals import joblib

class ClassifyObjects:

	def __init__(self):

		# Instance of RosPack with the default file path
		path = rospkg.RosPack().get_path('nimbus_perception')

		# Load the classifier model
		self.model = joblib.load(path + "/config/model.pkl")

		# initialize the service and pass in inputs
		self.service = rospy.Service('classify', Classify, self.handle_classify)

	def handle_classify(self, req):
	    
	    print "Returning [l=%f a=%f b=%f x=%f y=%f z=%f]"%(req.l, req.a, req.b, req.x, req.y, req.z)
	    str = self.model.predict([req.l, req.a, req.b, req.x, req.y, req.z])
	    return str[0]

if __name__ == "__main__":

	rospy.init_node('classify_server')
	classify = ClassifyObjects()
	print "Ready to classify."

	rospy.spin()