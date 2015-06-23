from jaus_message import JausMessage
from capra_jaus.msg import SetAuthority as SetAuthorityMsg
import rospy
import struct


class SetAuthority(JausMessage):
    code = 0x0001
    data_fmt = '<B'

    def __init__(self, message):
        self.parse(message.payload[0])
        self.message = message
        self.authorityCode_index = 0
        self.publisher = rospy.Publisher('jaus_access_control/set_authority', SetAuthorityMsg, queue_size=10)

    def execute(self):
        msg = SetAuthorityMsg()
        msg.authority_code = self.data[self.authorityCode_index]
        msg.source_id = self.message.get_source_id()
        self.publisher.publish(msg)


