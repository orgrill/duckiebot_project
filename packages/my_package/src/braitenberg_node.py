#!/usr/bin/env python3

import os
import yaml
import rospy
import cv2
import numpy

from typing import Tuple
from duckietown.dtros import DTROS, NodeType, ParamType, DTParam
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class BraitenbergNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(BraitenbergNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.vehicle_name = os.environ["VEHICLE_NAME"]

        # Image processing setup
        self.bridge = CvBridge()

        # NOTE: Define more colors here if needed. These keys are used throughout the code as an iterable
        # color_boundies structure
        #      Key: Color Name
        #      Value: Tuple. First element is minimum pixel values to be considered in range of the respective color
        #                    Second element is maximum pixel value 
        # TODO: Calibrate pixel thresholds
        self.color_boundries = {"red": ([10, 10, 100], [50, 60, 255]),
                                "blue": ([100, 10, 10], [255, 60, 70]),
                                "green": ([10, 100, 10], [50, 255, 50]),
                                "white": ([100, 100, 100], [255, 255, 255])}
        
        self.avoid_colors = ["red", "blue"]
        self.goal_color = "white"

        # Publishers
        self.wheel_pub = rospy.Publisher(f"{self.vehicle_name}/wheels_driver_node/wheels_cmd",
                                         WheelsCmdStamped,
                                         queue_size = 10)

        # Debug Publishers
        self.debug_color_pub = {color: rospy.Publisher(f"~debug/{color}", CompressedImage, queue_size=10) for color in self.color_boundries.keys()}

        # Subscribers
        self.camera_sub = rospy.Subscriber(f"{self.vehicle_name}/camera_node/image/compressed", 
                                           CompressedImage,
                                           self.process_image,
                                           buff_size=1000000,
                                           queue_size=1)


        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()

        # Get static parameters
        self._baseline = rospy.get_param('~baseline')
        self._radius = rospy.get_param('~radius')
        self._k = rospy.get_param('~k')
        # Get editable parameters
        self._gain = DTParam('~gain',
                             param_type=ParamType.FLOAT, 
                             min_value=0.0, 
                             max_value=3.0)
        self._trim = DTParam('~trim',
                             param_type=ParamType.FLOAT, 
                             min_value=0.0, 
                             max_value=3.0)
        self._limit = DTParam('~limit',
                              param_type=ParamType.FLOAT, 
                              min_value=0.0, 
                              max_value=1.0)

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        rospy.set_param(f"/{self.vehicle_name}/camera_node/exposure_mode", "off")

        self.log("Initialized")


    def process_image(self, image):
        """
        Callback function for self.camera_sub
        Process the incoming image from the camera topic
        """
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image)
        color_images = {color: self.detect_color(cv_image, color) for color in self.color_boundries.keys()}

        # Publish debug images
        # TODO: Data intensive. Use if_any_listeners flag before publishing
        for (color, image) in color_images.items():
            msg = CompressedImage()
            msg.data = cv2.imencode('.jpg', image)[1].tobytes()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            self.debug_color_pub[color].publish(msg)

        left = right = 1
        for color in (self.avoid_colors + [self.goal_color]):
            current_direction = max(left, right)
            left, right = self.calculate_move(color_images[color],
                                              current_direction,
                                              left,
                                              right,
                                              avoid = color in self.avoid_colors)

        self.send_wheel_cmd(left, right)


    def calculate_move(self, img, current_direction: float, left: float, right: float, avoid: bool) -> Tuple[float, float]:
        """
        Split image vertically into two halves. Test which side has the most color matched pixels 
        Calculate left, right speeds proportional to how close the obstacle is (number of color pixels)
        """
        mid = img.shape[1] // 2
        split_size = img.shape[0] * mid
        left_sum = numpy.sum(numpy.where(img[:, :mid, :] != [0, 0, 0])) // 3
        right_sum = numpy.sum(numpy.where(img[:, mid:, :] != [0, 0, 0])) // 3
        new_direction = max(left_sum + 1, right_sum + 1)  / split_size * 1.5

        if new_direction < current_direction or left_sum == right_sum:
            return left, right

        if left_sum > right_sum:
            left = new_direction
        else:
            right = new_direction

        # Swap if we're not avoiding. Need to move towards the given color
        if not avoid:
            left, right = right, left

        return left, right


    def detect_color(self, image, color: str):
        """
        Use CV2 to detect the given color
        Returns an image containing pixels that fall within the color boundry
        """

        color_lower, color_upper = self.color_boundries[color]

        mask = cv2.inRange(image, 
                           numpy.array(color_lower, dtype = "uint8"),
                           numpy.array(color_upper, dtype = "uint8"))
        

        return cv2.bitwise_and(image, image, mask = mask)
    

    def send_wheel_cmd(self, left: float, right: float):
        l, r = self.speedToCmd(left, right)
        self.wheel_pub.publish(WheelsCmdStamped(vel_left = l, vel_right = r))


##################################
#          TEMPLATE CODE         #
#       (I didn't write this)    #
##################################
    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the
        output velocities

        Applies the motor constant k to convert the deisred wheel speeds
        to wheel commands. Additionally, applies the gain and trim from
        the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left
                wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right
                wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be
                packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self._k
        k_l = self._k

        # adjusting k by gain and trim
        k_r_inv = (self._gain.value + self._trim.value) / k_r
        k_l_inv = (self._gain.value - self._trim.value) / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv
        u_l = speed_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r,
                                -self._limit.value,
                                self._limit.value)
        u_l_limited = self.trim(u_l,
                                -self._limit.value,
                                self._limit.value)

        return u_l_limited, u_r_limited


    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.vehicle_name)
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass


    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file


    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)
##################################
#        END TEMPLATE CODE       #
##################################

    def on_shutdown(self):
        self.wheel_pub.publish(WheelsCmdStamped(vel_left = 0, vel_right = 0))
        super(BraitenbergNode, self).on_shutdown()



if __name__ == '__main__':
    node = BraitenbergNode(node_name='braitenberg_node')
    rospy.spin()