#!/usr/bin/env python

import rospy
import socket
import struct
from std_msgs.msg import String


class JausClient:
    def __init__(self):
        rospy.init_node("jaus_client", log_level=rospy.DEBUG)

        #self.server_ip = rospy.get_param('judge_server_address', '127.0.0.1')
        #self.server_port = rospy.get_param('judge_server_port', 10000)
        self.ip = rospy.get_param('~jaus_node_address', '127.0.0.1')
        self.port = rospy.get_param('~jaus_node_port', 3794)  # 55555)
        self.multicast_address = rospy.get_param('~multicast_address')
        rospy.logdebug("binding to: %s:%s", self.ip, self.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.ip, self.port))
        if self.multicast_address is not None and len(self.multicast_address) > 0:
            group = socket.inet_aton(self.multicast_address)
            mreq = struct.pack('4sL', group, socket.INADDR_ANY)
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.publisher = rospy.Publisher('/jaus_agent/execute', String, queue_size=10)

    def start(self):
        try:
            while True:
                data, address = self.sock.recvfrom(4096)
                rospy.logdebug('received %s bytes from %s', len(data), address)
                rospy.logdebug(":".join("{:02x}".format(ord(c)) for c in data))

                if data:
                    s = String()
                    s.data = data
                    self.publisher.publish(s)
        except Exception as e:
            rospy.logerr(e.message)
            self.sock.close()

if __name__ == '__main__':
    jaus = JausClient()
    jaus.start()
