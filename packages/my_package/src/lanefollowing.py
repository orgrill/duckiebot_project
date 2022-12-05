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
    

    # this mask polygon describes the shape of the image that the robot cares about
    # this appears to be the most important to using this algorithm - modify at your own risk
    mask_poly = np.array([[
    [0, 480],
    [0, 480 - 100],
    [640 // 3, 480//2],
    [2*640//3, 480//2],
    [640, 480 - 100]
    [640, 480]                           
    ]])

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.vehicle_name = os.environ["VEHICLE_NAME"]

        # Image processing setup
        self.bridge = CvBridge()
        
        # Rospy Publishers
        self.wheel_pub = rospy.Publisher(f"{self.vehicle_name}/wheels_driver_node/wheels_cmd",
                                         WheelsCmdStamped,
                                         queue_size = 10)
        

        
        # these two publishers create the view interface
        self.debug_roboview = rospy.Publisher(f"~debug/roboview", CompressedImage, queue_size=10)
        self.debug_lineview = rospy.Publisher(f"~debug/lineview", CompressedImage, queue_size=10)

        

        # Subscribers  these are where we get sensor data for the robot's controls
        self.camera_sub = rospy.Subscriber(f"{self.vehicle_name}/camera_node/image/compressed", 
                                           CompressedImage,
                                           self.preprocess_image,
                                           buff_size=1000000,
                                           queue_size=1)
        

        # some helper variables
        self.theta_threshold = 5


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
        
        # print('raw width, height', width, height)  # debug

        # define region of interest, we don't care about anything not in here:

        
        # preprocess the image
        frame = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)  # convert to grayscale
        frame = cv2.GaussianBlur(frame, (5, 5), 0)          # apply a gaussian blur
        frame = cv2.Canny(frame, 50, 150)                   # apply the Canny filter to get the edges

        # now perform masking
        mask = np.zeros_like(frame)
        # write the mask poly onto our mask
        cv2.fillPoly(
            mask, LaneFollowNode.mask_poly, 255
        )

        frame = cv2.bitwise_and(frame, mask)
        cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # now send the original image and the pre-processed image to the image processor
        self.image_processor(
            original_image=cv_image,
            preprocessed_image=frame
        )


    
    def image_processor(self, original_image, preprocessed_image):

        width, height, layers = original_image.shape
        theta = 0


        # first find the lines in the image
        line_image = np.zeros_like(original_image)    # create blank image to draw lines on
        lines = cv2.HoughLinesP(
            preprocessed_image,
            1, np.pi/180,
            10,                                # this argument is min amount of pixels to get a line
            minLineLength=10, maxLineGap=100    # further tweaks for sensitivity
        )
    

        if lines is not None:
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                theta +=  np.arctan2((y2 - y1), (x2 - x1))

                # now let's draw these lines and publish them to the bot
                cv2.line(original_image, (x1, y1), (x2, y2), (0, 0, 255), 3)
                cv2.line(preprocessed_image, (x1, y1), (x2, y2), (255, 0, 0), 3)


        # put some masking lines over the debug views
        cv2.polylines(original_image, LaneFollowNode.mask_poly, True, (255, 0 ,0), 2)
        cv2.polylines(preprocessed_image, LaneFollowNode.mask_poly, True, (255, 0 ,0), 2)


        # publish everything to the debug panel - hard coded
        # this slows down performance egregiously
        # commented out for now
        for channel, screen in [(self.debug_roboview, preprocessed_image), 
                                (self.debug_lineview, original_image) 
                                
                                ]:

            msg = CompressedImage()
            msg.data = cv2.imencode('.jpg', screen)[1].tobytes()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            channel.publish(msg)

        self.send_wheel_cmd(*self.calculate_move(theta))



    def calculate_move(self, theta) -> Tuple[float, float]:
        """
        simple for robot behavior 
        

        rules based (for now)
        """
        if abs(theta) <= self.theta_threshold:
            # if the total magnitude of the theta deviation, run both wheels

            return (1, .9)  # trim straight travel a little bit
        else:
            # otherwise the sum of all the angles is greater than theta threshold apply the appropriate correction
            if theta >= self.theta_threshold:
                
                return (0, 1)
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