import rospy
from jaus_message import JausMessage


class QuerySubsystemList(JausMessage):
    code = 0x2b02
    data_fmt = ''

    def __init__(self, message):
        pass

    def execute(self):
        return ReportSubsystemList()

    def verify_query_message(self, query_message):
        try:
            if len(query_message) == 0:
                return 0
            else:
                raise Exception('query_message should be of length 0')
        except Exception as e:
            rospy.logerr(e.message)
        return 4


class ReportSubsystemList(JausMessage):
    code = 0x4b02
    data_fmt = '<HBB'

    def execute(self):
        self.pack((105, 1, 1))


