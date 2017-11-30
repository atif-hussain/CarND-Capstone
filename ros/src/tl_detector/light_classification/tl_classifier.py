from styx_msgs.msg import TrafficLight
import os
import rospy
import numpy as np
import cv2
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #simple classifier, nothing needed
        return

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # we apply color thresholds to the image, 
        # pre-applying 3x3 blur, thresholds can be very tight
        blur = cv2.blur(image,(3,3))

        #apply RED/GREEN/YELLOW masks
        red_count = np.sum(cv2.inRange(hsv, np.array([235,0,0]), np.array([255,20,20])))
        green_count = np.sum(cv2.inRange(hsv, np.array([0,235,0]), np.array([20,255,20])))
        yellow_count = np.sum(cv2.inRange(hsv, np.array([235,235,0]), np.array([255,255,20])))
        M = max(1, red_count, green_count, yellow_count)
        if (red_count = M) #biggest light is red
            rospy.loginfo("red light ahead")
            return TrafficLight.RED
        elif (yellow_count = M) #biggest light is yellow
            rospy.loginfo("yellow light ahead")
            return TrafficLight.YELLOW
        elif (green_count = M) #biggest light is green
            rospy.loginfo("green light ahead")
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
