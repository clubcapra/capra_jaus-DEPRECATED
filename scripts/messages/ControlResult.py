from jaus_message import JausMessage
import rospy
import struct

class ConfirmControl(JausMessage):
    code = 0x000f
    data_fmt = '<B'

    def __init__(self, response_code):
        self.response_code = response_code

    def execute(self):
        self.pack((self.response_code, ))

class RejectControl(JausMessage):
    code = 0x0010
    data_fmt = '<B'

    def __init__(self, response_code):
        self.response_code = response_code

    def execute(self):
        self.pack((self.response_code, ))