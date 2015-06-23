from jaus_message import JausMessage
from capra_jaus.srv import QueryEvents as QueryEventsService
import rospy
import struct


class QueryEvents(JausMessage):
    code = 0x21f0
    data_fmt = '<B'
    variant_index = 0
    message_id_index = 1
    event_type_index = 1
    event_id_index = 1
    all_events_index = 1

    def __init__(self, message):
        self.parse(message.payload[0])
        self.message = message

    def execute(self):
        stripped_data = self.message.payload[1:]
        message_id = event_type = event_id = all_events = 0
        variant = self.data[self.variant_index]
        if variant == 0:
            message_id = struct.unpack('<S', stripped_data)[0]
            rospy.logdebug('Querying events of message_id: %s', message_id)
        elif variant == 1:
            event_type = struct.unpack('<B', stripped_data)[0]
            rospy.logdebug('Querying events of event_type: %s', event_type)
        elif variant == 2:
            event_id = struct.unpack('<B', stripped_data)[0]
            rospy.logdebug('Querying events of event_id: %s', event_id)
        elif variant == 3:
            all_events = struct.unpack('<B', stripped_data)[0]
            rospy.logdebug('Querying all events: %s', all_events)
        else:
            rospy.logerr('The vtag_field of QueryEvents is not valid. Value: %s', variant)
        rospy.wait_for_service('jaus_query_events')
        query_events_call = rospy.ServiceProxy('jaus_query_events', QueryEventsService)
        queried_events = query_events_call(message_id, event_type, event_id, all_events)
        return ReportEvents(queried_events.events)

    def verify_query_message(self, query_message):
        try:
            self.parse(query_message)
            variant = self.data[self.variant_index]
            stripped_data = query_message[1:]
            if variant == 0:
                struct.unpack('<S', stripped_data)[0]
            elif variant == 1:
                struct.unpack('<B', stripped_data)[0]
            elif variant == 2:
                struct.unpack('<B', stripped_data)[0]
            elif variant == 3:
                struct.unpack('<B', stripped_data)[0]
            else:
                return 4
            return 0
        except Exception as e:
            rospy.logerr(e.message)
        return 4


class ReportEvents(JausMessage):
    code = 0x41f0
    data_fmt = '<B'

    def __init__(self, events):
        self.events = events

    def execute(self):
        self.pack((len(self.events), ))
        for event in self.events:
            self.output += struct.pack('<BBI', event.event_type, event.event_id, len(event.query_message))
            self.output += event.query_message
