#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import roslib
#roslib.load_manifest('vision')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from cv2 import aruco
from nav_msgs.msg import Odometry
from math import *
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
	
index=1
x = 0.0
y = 0.0
theta = 0.0
count_color=1
value=[]

def init_marker(index,x,y,colorR,colorG,colorB):

	marker_object = Marker()
	marker_object.header.frame_id = "/odom"
	marker_object.header.stamp    = rospy.get_rostime()
	marker_object.ns = "haro"
	marker_object.id = index
	marker_object.type = Marker.SPHERE
	marker_object.action = Marker.ADD
    
	my_point = Point()
	my_point.z = 0
	my_point.x= x
	my_point.y = y
	marker_object.pose.position = my_point
    
	marker_object.pose.orientation.x = 0
	marker_object.pose.orientation.y = 0
	marker_object.pose.orientation.z = 0.0
	marker_object.pose.orientation.w = 1.0
	marker_object.scale.x = 0.2
	marker_object.scale.y = 0.2
	marker_object.scale.z = 0.2

	marker_object.color.r = colorR
	marker_object.color.g = colorG
	marker_object.color.b = colorB
    # This has to be, otherwise it will be transparent
	marker_object.color.a = 1.0
        
    # If we want it for ever, 0, otherwise seconds before desapearing
	marker_object.lifetime = rospy.Duration(0)
	return marker_object


def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newLaser(msg):
    global value

    value = msg.ranges

class image_converter:


	def __init__(self): 
		self.image_pub = rospy.Publisher("image_topic_2",Image)

		self.bridge = CvBridge()
		self.mark = rospy.Publisher('/marker_basic', Marker, queue_size=1)

		#self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
		#self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.callback)

		self.odom = rospy.Subscriber("/odom", Odometry, newOdom)

		self.laser = rospy.Subscriber("/scan", LaserScan, newLaser)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		ratio=1 
		global x
		global y
		global theta 
		global count_color
		global value
		global index

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		lower_range_green = np.array([36, 25, 25])
		upper_range_green = np.array([86, 255,255])
		mask_green = cv2.inRange(hsv, lower_range_green, upper_range_green)


		low_yellow = np.array([20, 100, 100])
		high_yellow = np.array([30, 255, 255])
		mask_yellow = cv2.inRange(hsv, low_yellow, high_yellow)


		low_orange = np.array([15, 100, 100])
		high_orange = np.array([20, 255, 255])
		mask_orange = cv2.inRange(hsv, low_orange, high_orange)
		# lower mask (0-10)
		lower_red = np.array([0,50,50])
		upper_red = np.array([10,255,255])
		mask0 = cv2.inRange(hsv, lower_red, upper_red)

		# upper mask (170-180)
		lower_red = np.array([170,50,50])
		upper_red = np.array([180,255,255])
		mask1 = cv2.inRange(hsv, lower_red, upper_red)

		# join my masks
		mask_red = mask0+mask1

		kernel = np.ones((3, 3), np.uint8)




 
		if(count_color == 1 ): 
			erosion = cv2.erode(mask_green,kernel,iterations = 3)
			
		elif(count_color == 2 ): 
			erosion = cv2.erode(mask_yellow,kernel,iterations = 3)
			

		elif(count_color == 3 ): 
			erosion = cv2.erode(mask_orange,kernel,iterations = 10)
			

		elif(count_color == 4 ):
			erosion = cv2.erode(mask_red,kernel,iterations = 3)
			

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		thresh = cv2.threshold(erosion, 60, 255, cv2.THRESH_BINARY)[1]

		_,contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		for c in contours	:
			coulor="null" 	
			area = cv2.contourArea(c)
			shape = "unidentified"
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.02 * peri, True)
			if area<10:
				continue
			M = cv2.moments(c)
			cX = int((M["m10"] /( M["m00"]+0.0001)) * ratio)
			cY = int((M["m01"] / ( M["m00"]+0.0001)) * ratio)

			angle = (atan2(cY,cX))
			laser_emp= (3.14-(angle))/(3.14/len(value))
			#laser_emp= (4.18-(angle))/(4.18/len(value))
			print(laser_emp)
			dist = value[int(laser_emp)]

			xp=dist*cos(angle)
			yp=dist*sin(angle)

				
			xreal=(x+xp*cos(theta)+yp*sin(theta))
			yreal=(y+yp*cos(theta)+xp*sin(theta))
			print(dist)


			if len(approx) == 3:
				shape = "triangle"
				if(count_color == 2): 
					print("Radioactive")
					self.mark.publish(init_marker(index,xreal,yreal,1,1,0))

					index+=1
				elif(count_color==4):
					print("Danger")
					self.mark.publish(init_marker(index,xreal,yreal,0.0,0.1,1.0))
					index+=1
		
		# if the shape has 4 vertices, it is either a square or
		# a rectangle
			elif len(approx) == 4:
		# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
				(x, y, w, h) = cv2.boundingRect(approx)
				ar = w / float(h)
				if(count_color == 2): 
					print("Biohazard")
					self.mark.publish(init_marker(index,xreal,yreal,1,0.9,0.2))
					index+=1
				elif(count_color == 3): 
					print("Fire")
					self.mark.publish(init_marker(index,xreal,yreal,1,0.6,0))
					index+=1							
				elif(count_color==4):
					print("Toxic")
					self.mark.publish(init_marker(index,xreal,yreal,1,0.2,1.0))
					index+=1



						# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
				shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"


		# if the shape is a pentagon, it will have 5 vertices
			elif len(approx) == 5:
				shape = "pentagon"
				if(count_color == 2): 
					print("Radioactive")
					self.mark.publish(init_marker(index,xreal,yreal,1,1,0))
					index+=1
		# otherwise, we assume the shape is a circle
			else:
				shape = "circle"
				if(count_color == 1):
					print("Alive worker")
					self.mark.publish(init_marker(index,xreal,yreal,0,1,0))
					index+=1
				elif(count_color==4):
					print("Dead Worker")
					self.mark.publish(init_marker(index,xreal,yreal,1,0,0))
					index+=1
				if(count_color == 2): 
					print("Radioactive")
					self.mark.publish(init_marker(index,xreal,yreal,1,1,0))
					print(laser_emp)
					#print(angle)
					print(dist)
					index+=1
		# return the name of the shape
			# compute the center of the contour, then detect the name of the
	# shape using only the contour


	# multiply the contour (x, y)-coordinates by the resize ratio,
	# then draw the contours and the name of the shape on the image
			c = c.astype("float")
			c *= ratio
			c = c.astype("int")
			cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
			cv2.putText(cv_image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)
	# show the output image
		cv2.imshow("Image", cv_image)

		cv2.imshow("Image2",thresh)

		count_color+=1
		if(count_color==5):
			count_color=1
		

		cv2.waitKey(20)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):


	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)







