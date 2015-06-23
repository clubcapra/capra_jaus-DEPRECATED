from capra_jaus.srv import CreateEvent as CreateEventService
import rospy
from jaus_message import JausMessage
from EventRequestResult import *
import struct


class CreateEvent(JausMessage):
    code = 0x01f0
    data_fmt = '<BBHI'
    request_id_index = 0
    event_type_index = 1
    requested_rate_index = 2
    query_message_length_index = 3

    def __init__(self, message):
        self.message = message
        self.parse(message.payload[0:8])

    def execute(self):
        ros_judp_message = self.message.to_ros_judp_message()
        query_message = ros_judp_message.data[8:8+self.data[self.query_message_length_index]]
        rospy.wait_for_service('jaus_create_event')
        create_event_call = rospy.ServiceProxy('jaus_create_event', CreateEventService)
        create_event_result = create_event_call(self.data[self.request_id_index], self.data[self.event_type_index], self.data[self.requested_rate_index], query_message, ros_judp_message)

        rospy.logdebug('Created event with request_id:%s, event_type:%s, requested_rate:%s, query_message:%s, judp_message:%s', self.data[self.request_id_index], self.data[self.event_type_index], self.data[self.requested_rate_index], ":".join("{:02x}".format(ord(c)) for c in query_message), ros_judp_message)

        if create_event_result.is_accepted:
            rospy.logdebug('Creation accepted.')
            result = ConfirmEventRequest(create_event_result.confirmed_event_request)
        else:
            rospy.logdebug('Creation rejected.')
            result = RejectEventRequest(create_event_result.rejected_event_request)
        return result

    def verify_query_message(self, query_message):
        try:
            data = struct.unpack(self.data_fmt, query_message)
            if len(query_message) == 8 + data[self.query_message_length_index]:
                return 0
            else:
                raise Exception("query_message is not of right length")
        except Exception as e:
            rospy.logerr(e.message)
        return 4
