import rospy
from jaus_message import JausMessage


class QueryIdentification(JausMessage):
    code = 0x2b00
    data_fmt = '<B'
    query_type_index = 0

    def __init__(self, message):
        self.parse(message.payload)
        self.component_id = message.destination_id & 0xFF

    def execute(self):
        query_type = self.data[self.query_type_index]
        if query_type == 0:
            rospy.logdebug('QueryIdentification, query_type = Reserved (0)')
        elif query_type == 1:
            rospy.logdebug('QueryIdentification, query_type = System Identification (1)')
        elif query_type == 2:
            rospy.logdebug('QueryIdentification, query_type = Subsystem Identification (2)')
        elif query_type == 3:
            rospy.logdebug('QueryIdentification, query_type = Node Identification (3)')
        elif query_type == 4:
            rospy.logdebug('QueryIdentification, query_type = Component Identification (4)')
        else:
            rospy.logdebug('QueryIdentification, query_type = Reserved (5 to 255)')
        return ReportIdentification(query_type, self.component_id)

    def verify_query_message(self, query_message):
        try:
            self.parse(query_message)
            if self.data[self.query_type_index] > 4 and self.data[self.query_type_index] < 255 \
                    or self.data[self.query_type_index] == 0:
                return 4
            return 0
        except Exception as e:
            rospy.logerr(e.message)
        return 4

class ReportIdentification(JausMessage):
    code = 0x4b00
    data_fmt = '<BHB'
    query_type_index = 0
    type_index = 1
    string_length = 2
    string = 3

    def __init__(self, query_type, component_id):
        self.query_type = query_type
        self.component_id = component_id

    def execute(self):
        if self.query_type == 0:
            rospy.logdebug("ReportIdentification: Reserved (0)")
            identification = "Reserved Identification"
        elif self.query_type == 1:
            rospy.logdebug("ReportIdentification: System Identification (1)")
            identification = "IGVC IOP Challenge"
        elif self.query_type == 2:
            rospy.logdebug("ReportIdentification: Subsystem Identification (2)")
            identification = "Capra Jaus"
        elif self.query_type == 3:
            rospy.logdebug("ReportIdentification: Node Identification (3)")
            identification = "Capra Jaus Node"
        elif self.query_type == 4:
            rospy.logdebug("ReportIdentification: Component Identification (4) ComponentId: %s", self.component_id)
            if self.component_id == 1:
                identification = "Platform Management, Navigation and Reporting"
            else:
                identification = "Invalild Component Id"
        else:
            rospy.logdebug("ReportIdentification: Reserved (5-255)")
            identification = "Reserved Identification"
        self.pack((self.query_type, 10001, len(identification)))
        self.output += identification
