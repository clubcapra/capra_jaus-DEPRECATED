import rospy
from jaus_message import JausMessage
import struct


class TriggerEvent(JausMessage):
    data_fmt = '<BBI'
    event_id_index = 0
    sequence_id_index = 1
    query_message_length_index = 2

    def __init__(self, input_jdp_message):
        rospy.logdebug("Triggering an event with payload %s", ":".join("{:02x}".format(ord(c)) for c in input_jdp_message.payload))
        self.parse(input_jdp_message.payload[0:6])
        self.message = input_jdp_message.payload

    def execute(self):
        event_id = self.data[self.event_id_index]
        sequence_number = self.data[self.sequence_id_index]
        query_message_length = self.data[self.query_message_length_index]
        query_message = self.message[6:6 + query_message_length]
        return event_id, sequence_number, query_message


class ExecuteEvent(JausMessage):
    code = 0x41f1
    data_fmt = '<BBIH'

    def __init__(self, event_id, sequence_number, return_message):
        self.event_id = event_id
        self.sequence_number = sequence_number
        self.return_message = return_message

    def execute(self):
        self.return_message.execute()
        self.pack((self.event_id, self.sequence_number, len(self.return_message.output) + 2, self.return_message.code))
        self.output += self.return_message.output
