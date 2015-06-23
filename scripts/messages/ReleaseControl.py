from jaus_message import JausMessage
from capra_jaus.srv import ReleaseControl as ReleaseControlService
from ControlResult import RejectControl
import rospy
import struct


class ReleaseControl(JausMessage):
    code = 0x000e
    data_fmt = ''

    def __init__(self, message):
        self.message = message

    def execute(self):
        rospy.wait_for_service('jaus_release_control')
        release_control_call = rospy.ServiceProxy('jaus_release_control', ReleaseControlService)
        release_control_result = release_control_call(self.message.get_source_id())
        return RejectControl(release_control_result.response_code)


