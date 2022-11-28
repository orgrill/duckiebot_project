#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.srv import ChangePattern, SetCustomLEDPatternRequest
from std_msgs.msg import String
from collections import deque

class ServiceMessage:
    def __init__(self, data):
        self.data = data

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.led_svc = f"{os.environ['VEHICLE_NAME']}/led_emitter_node/set_pattern"
        self.change_led = rospy.ServiceProxy(self.led_svc, ChangePattern)
        #self.change_led = rospy.ServiceProxy("~set_pattern", ChangePattern)

    def run(self):
        # publish message every 1 second
        rospy.wait_for_service(self.led_svc)
        rate = rospy.Rate(1) # 1Hz
        colors = deque(['GREEN', 'RED', 'BLUE', 'WHITE'])
        while not rospy.is_shutdown():
            colors.rotate(1)
            self.change_led(ServiceMessage(colors[0]))
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
