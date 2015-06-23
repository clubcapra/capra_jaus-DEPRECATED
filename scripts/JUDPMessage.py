import struct
import rospy
from capra_jaus.msg import JUDPMessage as JUDPMessageROS

class JUDPMessage:
    sequence_counter = 0
    fixed_source_id = 0x00690101

    def __init__(self):
        self.version = 0x2
        self.message_type = 0
        self.hc_flags = 0
        self.data_size = 0
        self.hc_number = 0
        self.hc_length = 0
        self.pbad_flags = 0
        self.destination_id = 0
        self.source_id = 0
        self.message_code = 0
        self.payload = None
        self.sequence_number = 0
        self.output = None

    def parse(self, data):
        self.version, self.type_hc = struct.unpack("<BB", data[0:2])
        self.message_type = self.type_hc >> 2
        self.hc_flags = self.type_hc % 0b11
        self.data_size = struct.unpack("<H", data[2:4])[0] - 2  # remove 2 because the first 2 bytes are the message code
        rospy.logdebug("Data size: %s", self.data_size)
        index_modifier = 0
        if self.hc_flags != 0:
            self.hc_number, self.hc_length = struct.unpack("<BB", data[4:6])
            index_modifier += 2
        self.pbad_flags, self.destination_id, self.source_id, self.message_code = struct.unpack("<BIIH", data[4 + index_modifier:15 + index_modifier])
        self.payload = data[15 + index_modifier: 15 + index_modifier + self.data_size - 14]
        self.sequence_number = struct.unpack("<H", data[-2::])[0]

    def pack(self, message_code, payload):
        self.output = struct.pack("<BBHBIIH", self.version, 0x00, len(payload) + 16, 0x01, self.destination_id, self.fixed_source_id, message_code)
        self.output += payload
        JUDPMessage.sequence_counter += 1
        self.output += struct.pack("<H", JUDPMessage.sequence_counter)

    def get_message_type(self):
        return self.message_type

    def get_hc_flags(self):
        return self.hc_flags

    def get_data_size(self):
        return self.data_size

    def get_hc_number(self):
        return self.hc_number

    def get_hc_length(self):
        return self.hc_length

    def get_priority(self):
        return (self.pbad_flags & 0b11000000) >> 6

    def get_broadcast_flags(self):
        return (self.pbad_flags & 0b00110000) >> 4

    def get_ack_nack_flags(self):
        return (self.pbad_flags & 0b00001100) >> 2

    def get_data_flags(self):
        return self.pbad_flags & 0b00000011

    def get_destination_id(self):
        return self.destination_id

    def get_source_id(self):
        return self.source_id

    def get_payload(self):
        return self.payload

    def get_sequence_number(self):
        return self.sequence_number

    def dump(self):
        pass

    def __str__(self):
        s = "SDP:\n"
        s += " Message type: %d\n" % self.get_message_type()
        s += " HC flags: %d\n" % self.get_hc_flags()
        s += " Data size: %d\n" % self.get_data_size()
        if self.get_hc_flags() != 0:
            s += " HC Number: %d\n" % self.get_hc_number()
            s += " HC Length: %d\n" % self.get_hc_length()
        s += " Priority: %d\n" % self.get_priority()
        s += " Broadcast flags: %d\n" % self.get_broadcast_flags()
        s += " Ack/Nack flags: %d\n" % self.get_ack_nack_flags()
        s += " Data flags: %d\n" % self.get_data_flags()
        s += " Destination ID: %s\n" % self.get_destination_id()
        s += " Source ID: %s\n" % self.get_source_id()
        s += " Message Code: %s\n" % self.message_code
        s += " Payload: %s\n" % ":".join("{:02x}".format(ord(c)) for c in self.get_payload())
        s += " Sequence number: %d" % self.get_sequence_number()
        return s

    def to_ros_judp_message(self):
        message = JUDPMessageROS()
        message.version = self.version
        message.type_hc = self.type_hc
        message.size = self.get_data_size()
        message.pbad_flags = self.pbad_flags
        message.destination_id = self.get_destination_id()
        message.source_id = self.get_source_id()
        message.message_code = self.message_code
        message.data = self.get_payload()
        message.sequence_number = self.get_sequence_number()
        return message

