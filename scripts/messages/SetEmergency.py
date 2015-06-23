import rospy
from jaus_message import JausMessage
from std_msgs.msg import UInt8, UInt16
from jaus_enums import *
from PublisherFactory import PublisherFactory


class SetEmergency(JausMessage):
    # as5710 p40
    code = 0x6
    data_fmt = '<H'

    def __init__(self, message):
        self.parse(message.payload)
        self.emergency_code = self.data[0]
        self.publisher_su = PublisherFactory.get_publisher("/jaus_agent/status_update", UInt8, 10)
        self.publisher_e = PublisherFactory.get_publisher("/jaus_agent/emergency", UInt16, 10)

    def execute(self):
        rospy.logdebug('SetEmergency code %s', self.emergency_code)
        if self.emergency_code in [EmergencyCode.STOP]:
            s_status = UInt8()
            s_status.data = Status.EMERGENCY
            s_emergency = UInt16()
            s_emergency.data = self.emergency_code
            self.publisher_su.publish(s_status)
            self.publisher_e.publish(s_emergency)