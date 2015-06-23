#!/usr/bin/env python

import rospy
import struct
from std_msgs.msg import String

def listener():
    rospy.init_node("jaus_event", log_level=rospy.DEBUG)
    rate = rospy.get_param('~jaus_event_rate', 1) 
    message = str(rospy.get_param('~jaus_event_message', ""))
    rospy.logdebug("Starting a jaus_event_node with parameters: rate:%s, message:%s", rate, message)
    s = String()
    s.data = message[:-1].decode("hex")
    publisher = rospy.Publisher('/jaus_agent/execute_event', String, queue_size=10)
    r = rospy.Rate(60/rate)
    run_count = 0
    while not rospy.is_shutdown():
        rospy.logdebug("Event publishing %s", ":".join("{:02x}".format(ord(c)) for c in s.data))
        s.data = s.data[:16] + struct.pack('<B', run_count) + s.data[17:]
        publisher.publish(s)
        run_count += 1
        if run_count > 255:
            run_count = 0
        r.sleep()

listener()
