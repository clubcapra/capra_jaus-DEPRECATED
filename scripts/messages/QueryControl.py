from jaus_message import JausMessage
from capra_jaus.srv import QueryControl as QueryControlService
import rospy
import struct


class QueryControl(JausMessage):
    code = 0x200d
    data_fmt = ''

    def __init__(self, message):
        pass

    def execute(self):
        rospy.wait_for_service('jaus_query_control')
        query_control_call = rospy.ServiceProxy('jaus_query_control', QueryControlService)
        query_control_result = query_control_call()
        return ReportControl(query_control_result.subsystem_id,
                             query_control_result.node_id,
                             query_control_result.component_id,
                             query_control_result.authority_code)

class ReportControl(JausMessage):
    code = 0x400d
    data_fmt = '<HBBB'

    def __init__(self, subsys_id, node_id, component_id, authority_code):
        self.subsys_id = subsys_id
        self.node_id = node_id
        self.component_id = component_id
        self.authority_code = authority_code
    def execute(self):
        self.pack((self.subsys_id, self.node_id, self.component_id, self.authority_code, ))