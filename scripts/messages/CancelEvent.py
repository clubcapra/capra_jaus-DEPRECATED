from capra_jaus.srv import CancelEvent as CancelEventService
import rospy
import struct
from jaus_message import JausMessage
from EventRequestResult import *


class CancelEvent(JausMessage):
    code = 0x01f2
    data_fmt = '<BB'
    request_id_index = 0
    event_id_index = 1

    def __init__(self, message):
        self.message = message
        self.parse(message.payload)

    def execute(self):
        rospy.wait_for_service('jaus_cancel_event')
        cancel_event_call = rospy.ServiceProxy('jaus_cancel_event', CancelEventService)
        cancel_event_result = cancel_event_call(self.data[self.request_id_index], self.data[self.event_id_index])

        rospy.logdebug('Canceled event with request_id:%s, event_id:%s', self.data[self.request_id_index],
                       self.data[self.event_id_index])

        if cancel_event_result.is_accepted:
            rospy.logdebug('Cancellation accepted.')
            result = ConfirmEventRequest(cancel_event_result.confirmed_event_request)
        else:
            rospy.logdebug('Cancellation rejected.')
            result = RejectEventRequest(cancel_event_result.rejected_event_request)
        return result

    def verify_query_message(self, query_message):
        try:
            struct.unpack(self.data_fmt, query_message)
            return 0
        except Exception as e:
            rospy.logerr(e.message)
        return 4
