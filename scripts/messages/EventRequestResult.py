from jaus_message import JausMessage
import rospy


class ConfirmEventRequest(JausMessage):
    code = 0x01f3
    data_fmt = '<BBH'

    def __init__(self, confirmed_event_request):
        self.confirmed_event_request = confirmed_event_request

    def execute(self):
        self.pack((self.confirmed_event_request.request_id, self.confirmed_event_request.event_id, self.confirmed_event_request.confirmed_rate))


class RejectEventRequest(JausMessage):
    code = 0x01f4
    data_fmt = '<BBB'

    def __init__(self, rejectedEventRequest):
        self.rejectedEventRequest = rejectedEventRequest

    def execute(self):
        self.pack((self.rejectedEventRequest.presence_vector, self.rejectedEventRequest.request_id, self.rejectedEventRequest.response_code))
