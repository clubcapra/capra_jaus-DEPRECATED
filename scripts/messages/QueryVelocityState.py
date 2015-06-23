from jaus_message import JausMessage
import struct
import rospy


class QueryVelocityState(JausMessage):
    code = 0x2404
    data_fmt = '<H'

    def __init__(self, input_judp_message):
        self.parse(input_judp_message.payload)

    def execute(self):
        return ReportVelocityState(self.data[0])

    def verify_query_message(self, query_message):
        try:
            struct.unpack(self.data_fmt, query_message)
            return 0
        except Exception as e:
            rospy.logerr(e.message)
        return 4

class ReportVelocityState(JausMessage):
    code = 0x4404
    data_fmt = '<H'
    max_ushort = 65535
    max_uint = 4294967295

    def __init__(self, presence_vector):
        self.presence_vector = presence_vector
        required = 0x0141
        result = self.presence_vector & required
        self.parse(result)
        if result & 0x1:
            self.output += struct.pack('<I', self.max_uint/2)  # velocity_x
        if (result >> 1) & 0x1:
            self.output += struct.pack('<I', self.max_uint/2)  # velocity_y
        if (result >> 2) & 0x1:
            self.output += struct.pack('<I', self.max_uint/2)  # velocity_z
        if (result >> 3) & 0x1:
            self.output += struct.pack('<I', self.max_uint)  # velocity_rms
        if (result >> 4) & 0x1:
            self.output += struct.pack('<H', self.max_ushort/2)  # roll rate
        if (result >> 5) & 0x1:
            self.output += struct.pack('<H', self.max_ushort/2)  # Pitch Rate
        if (result >> 6) & 0x1:
            self.output += struct.pack('<H', self.max_ushort/2)  # Yaw Rate
        if (result >> 7) & 0x1:
            self.output += struct.pack('<H', self.max_ushort/2)  # rate_rms
        if (result >> 8) & 0x1:
            self.output += struct.pack('<I', self.max_ushort/2)  # timestamp
