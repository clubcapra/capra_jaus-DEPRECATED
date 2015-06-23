import rospy
from jaus_message import JausMessage


class QueryConfiguration(JausMessage):
    code = 0x2b01
    data_fmt = '<B'
    query_type_index = 0

    def __init__(self, message):
        self.parse(message.payload)

    def execute(self):
        query_type = self.data[self.query_type_index]
        rospy.logdebug("Querying %s Configuration", query_type)
        return_message = ReportConfiguration(query_type)
        return return_message

    def verify_query_message(self, query_message):
        try:
            self.parse(query_message)
            return 0
        except Exception as e:
            rospy.logerr(e.message)
        return 4


class ReportConfiguration(JausMessage):
    code = 0x4b01
    data_fmt = '<BBBBB'

    def __init__(self, query_type):
        self.query_type = query_type

    def execute(self):
        # We don't actually care about the query type because we only have one node in our only subsystem
        #if self.query_type == 2:  # Subsystem
        #elif self.query_type == 3: # Node
        # number of nodes, nodeId, number of components, component_id, instance_id
        self.pack((1, 1, 1, 1, 0))


