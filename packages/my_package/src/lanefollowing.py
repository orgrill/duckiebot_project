#!/usr/bin/env python3

import os
import yaml
import rospy
import cv2
import numpy as np

from typing import Tuple
from duckietown.dtros import DTROS, NodeType, ParamType, DTParam
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class LaneFollowNode(DTROS):
    
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.vehicle_name = os.environ["VEHICLE_NAME"]

        # Image processing setup
        self.bridge = CvBridge()
        

        # Publishers
        self.wheel_pub = rospy.Publisher(f"{self.vehicle_name}/wheels_driver_node/wheels_cmd",
                                         WheelsCmdStamped,
                                         queue_size = 10)
        
        self.cam_pub = rospy.Publisher(f'/{self.vehicle_name}/corrected_image/compressed', 
                                        CompressedImage, 
                                        queue_size=10)


        # Subscribers
        self.camera_sub = rospy.Subscriber(f"{self.vehicle_name}/camera_node/image/compressed", 
                                           CompressedImage,
                                           self.preprocess_image,
                                           buff_size=1000000,
                                           queue_size=1)
        

        # some helper variables
        self.x_intercepts = []  # container for our line segment x intercepts


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


    def preprocess_image(self, image):
        """
        Callback function for self.camera_sub
        Process the incoming image from the camera topic, apply the appropriate filters etc. then send
        the image to the "image processor" function
        """

        cv_image = self.bridge.compressed_imgmsg_to_cv2(image)



        width, height, layers = cv_image.shape  # get the shape of the image, we'll use this 
        
        # define region of interest, we don't care about anything not in here:
        mask_poly = np.array([[
            [0, height],                                    # bottom left pixel
            [width//3, height//2],                          # top left pixel
            [2*width//3, height//2],                        # top right pixel
            [width, height]                                 # bottom right pixel
        ]])
        
        # preprocess the image
        frame = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)  # convert to grayscale
        frame = cv2.GaussianBlur(frame, (5, 5), 0)          # apply a gaussian blur
        frame = cv2.Canny(frame, 50, 150)                   # apply the Canny filter to get the edges

        # now perform masking
        mask = np.zeros_like(frame)
        # write the mask poly onto our mask
        cv2.fillPoly(
            mask, mask_poly, 255
        )

        frame = cv2.bitwise_and(frame, mask)

        # now send the original image and the pre-processed image to the image processor
        self.image_processor(
            original_image=cv_image,
            preprocessed_image=frame
        )


    
    def image_processor(self, original_image, preprocessed_image):

        width, height, layers = original_image.shape
        
        # first find the lines in the image
        line_image = np.zeros_like(original_image)    # create blank image to draw lines on
        lines = cv2.HoughLinesP(
            preprocessed_image,
            1, np.pi/180,
            100,                                # this argument is min amount of pixels to get a line
            minLineLength=30, maxLineGap=100    # further tweaks for sensitivity
        )



        if lines is not None:

            for line in lines:
                x1, y1, x2, y2 = line[0]

                if (x2 - x1 == 0):
                    # handle vertical lines - we don't want to divide by zero
                    self.x_intercepts.append(x2)
                else:
                    # otherwise, calculate the x intercept
                    slope = (y2 - y1) / (x2 - x1)
                    b = y1 - slope*x1

                    # y = mx + b
                    # y / m = x + b/m
                    # 0 / m = x + b / m
                    # x = - b / m

                    x_intercept = -b / slope
                    self.x_intercepts.append(x_intercept)

                # draw the original line
                cv2.line(
                    original_image,
                    (x1, y1), (x2, y2),
                    (255, 0, 0)
                )

                # plot the average intercept now
                average_intercept = np.average(self.x_intercepts)
                
                cv2.circle(
                    original_image,                                 # plot them on the original image
                    (int(average_intercept), height),               # cast the average to the int and plot it at the bottom of the screen
                    30,                                             # radius of circle
                    (0, 0, 255),                                    # make it red (opencv is BGR color scheme)
                    -1                                              # -1 means it's a filled in circle
                )
        


        color_images = {color: self.detect_color(cv_image, color) for color in self.color_boundries.keys()}

        # Publish debug images
        # TODO: Data intensive. Use if_any_listeners flag before publishing
        for (color, image) in color_images.items():
            msg = CompressedImage()
            msg.data = cv2.imencode('.jpg', image)[1].tobytes()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            self.debug_color_pub[color].publish(msg)

        # every 5 x intercepts, pop off the oldest one
        if len(self.x_intercepts) % 5 == 0 and self.x_intercepts:
            self.x_intercepts.pop(0)
        

        if self.x_intercepts:

            left, right = self.calculate_move(np.average(self.x_intercepts), (width, height))
            print(left, right)

            self.send_wheel_cmd(left, right)



    def calculate_move(self, average_intercept, screensize) -> Tuple[float, float]:
        """
        simple for robot behavior 
        

        rules based (for now)
        """
        
        left_limit, right_limit = screensize
        throw = (left_limit / right_limit)

        # dirt simple bangon bang off controller
        position = average_intercept / throw
        if position > 0.5:
            return (0, 1)
        elif position == 0.5:
            return (1, 1)
        else:
            return (1, 0)


    

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
        super(LaneFollowNode, self).on_shutdown()



if __name__ == '__main__':
    node = LaneFollowNode(node_name='lane_following_node')
    rospy.spin()