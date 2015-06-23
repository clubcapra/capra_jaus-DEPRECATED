import rospy
from jaus_message import JausMessage
from capra_jaus.srv import QueryStatus as QueryStatusService


class QueryStatus(JausMessage):
    # as5710 p46
    code = 0x2002
    data_fmt = '<H'

    def __init__(self, message):
        pass

    def execute(self):
        rospy.logdebug('Receiving command QueryStatus')
        rospy.wait_for_service('jaus_query_status')
        query_status_call = rospy.ServiceProxy('jaus_query_status', QueryStatusService)
        query_status_result = query_status_call()
        rospy.logdebug('JAUS current status is %d', query_status_result.status)
        return ReportStatus(query_status_result.status)

    def verify_query_message(self, query_message):
        if len(query_message == 0):
            return 0
        return 4


class ReportStatus(JausMessage):
    # as5710 p51
    code = 0x4002
    data_fmt = '<BI'

    def __init__(self, status):
        self.status = status
        self.reserved = 0

    def execute(self):
        self.pack((self.status, self.reserved))