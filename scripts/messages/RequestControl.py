from jaus_message import JausMessage
from capra_jaus.srv import RequestControl as RequestControlService
from ControlResult import ConfirmControl
import rospy
import struct


class RequestControl(JausMessage):
    code = 0x000d
    data_fmt = '<B'

    def __init__(self, message):
        self.parse(message.payload[0])
        self.message = message
        self.authorityCode_index = 0

    def execute(self):
        rospy.wait_for_service('jaus_request_control')
        request_control_call = rospy.ServiceProxy('jaus_request_control', RequestControlService)
        request_control_result = request_control_call(self.data[self.authorityCode_index], self.message.get_source_id())
        return ConfirmControl(request_control_result.response_code)
