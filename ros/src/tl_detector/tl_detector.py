#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf

import cv2
import math
import numpy as np
import yaml


STATE_COUNT_THRESHOLD = 1


class TrafficLightInfo:
    def __init__(self, x, y, state):
        self.x = x
        self.y = y
        self.state = state


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.ImageID = 1
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.saved_tl_info = TrafficLightInfo(0, 0, TrafficLight.UNKNOWN)
        self.state_count = 0
        self.light_classifier = TLClassifier()

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', TrafficLight, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_traffic_light(self, pose):
        """Identifies the closest traffic light to the given position
        Args:
            pose (Pose): 3D position to match a light to

        Returns:
            int: index of the closest traffic light

        """
        q = pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        limit_angle = math.pi / 2
        max_distance = 150
        min_distance = float("inf")
        traffic_light_idx = -1
        for idx, item in enumerate(self.traffic_light_labels):
            heading = math.atan2((item.pose.pose.position.y - pose.position.y), (item.pose.pose.position.x - pose.position.x))
            if abs(yaw - heading) < limit_angle:
                distance = math.sqrt((item.pose.pose.position.x - pose.position.x) ** 2 + (item.pose.pose.position.y - pose.position.y) ** 2)
                # ignore traffic light if it is too far
                if distance <= max_distance:
                    if distance <= min_distance:
                        traffic_light_idx = idx
                        min_distance = distance
                    else:
                        break

        #rospy.loginfo("traffic_light_idx = %d", traffic_light_idx)
        return traffic_light_idx

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """
        #fx = self.config['camera_info']['focal_length_x']
        #fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        pos = self.pose.pose.position
        t = tf.transformations.translation_matrix((-pos.x, -pos.y, -pos.z))
        ort = self.pose.pose.orientation
        r = tf.transformations.quaternion_matrix((ort.x, ort.y, ort.z, -ort.w))

        # Camera seems to be a bit off, so apply some pitch & yaw
        r_camera = tf.transformations.euler_matrix(0, 0.16, 0.01)
        t_camera = tf.transformations.translation_matrix((0, 0.5, 0))
        # Combine all matrices
        m = tf.transformations.concatenate_matrices(r_camera, t_camera, r, t)
        # Make coordinate homogenous
        p = np.append(point_in_world, 1.0)
        # Transform the world point to camera coordinates
        tp = m.dot(p)
        # Project point to image plane
        # Note: the "correction" multipliers are tweaked by hand
        x = 3.5 * tp[1] / tp[0]
        y = 3.5 * tp[2] / tp[0]
        # Map camera image point to pixel coordinates
        x = int((0.5 - x) * image_width)
        y = int((0.5 - y) * image_height)
        # X-coordinate is the distance to the TL
        distance = tp[0]

        return (x, y, distance)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        '''for easier & accurate light recognition, 
           we can use traffic light's location's projection in view-field, 
           and only search for traffic light within a box around it '''

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        #TODO find the closest visible traffic light (if one exists)
        light_wp = self.get_closest_traffic_light(self.pose.pose)

        if light_wp:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
