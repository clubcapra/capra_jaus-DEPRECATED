from capra_jaus.srv import UpdateEvent as UpdateEventService
import rospy
import struct
from jaus_message import JausMessage
from EventRequestResult import *


class UpdateEvent(JausMessage):
    code = 0x01f1
    data_fmt = '<BBHBI'
    request_id_index = 0
    event_type_index = 1
    requested_rate_index = 2
    event_id_index = 3
    query_message_length_index = 4

    def __init__(self, message):
        self.message = message
        self.parse(message.payload[0:9])

    def execute(self):
        ros_judp_message = self.message.to_ros_judp_message()
        query_message = ros_judp_message.data[9:9+self.data[self.query_message_length_index]]
        rospy.wait_for_service('jaus_update_event')
        update_event_call = rospy.ServiceProxy('jaus_update_event', UpdateEventService)
        rospy.logdebug('Updating event with event_id:%s, request_id:%s, event_type:%s, requested_rate:%s, query_message:%s, judp_message:%s', self.data[self.event_id_index], self.data[self.request_id_index], self.data[self.event_type_index], self.data[self.requested_rate_index], ":".join("{:02x}".format(ord(c)) for c in query_message), ros_judp_message)
        update_event_result = update_event_call(self.data[self.event_id_index], self.data[self.request_id_index],
                                                self.data[self.event_type_index], self.data[self.requested_rate_index],
                                                query_message, ros_judp_message)

        rospy.logdebug('Updated event with event_id:%s, request_id:%s, event_type:%s, requested_rate:%s, query_message:%s, judp_message:%s', self.data[self.event_id_index], self.data[self.request_id_index], self.data[self.event_type_index], self.data[self.requested_rate_index], ":".join("{:02x}".format(ord(c)) for c in query_message), ros_judp_message)

        if update_event_result.is_accepted:
            rospy.logdebug('Update accepted.')
            result = ConfirmEventRequest(update_event_result.confirmed_event_request)
        else:
            rospy.logdebug('Update rejected.')
            result = RejectEventRequest(update_event_result.rejected_event_request)
        return result

    def verify_query_message(self, query_message):
        try:
            data = struct.unpack(self.data_fmt, query_message)
            if len(query_message) == 9 + data[self.query_message_length_index]:
                return 0
            else:
                raise Exception("query_message is not of right length")
        except Exception as e:
            rospy.logerr(e.message)
        return 4
