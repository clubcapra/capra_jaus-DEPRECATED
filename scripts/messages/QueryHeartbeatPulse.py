import rospy
from jaus_message import JausMessage


class QueryHeartbeatPulse(JausMessage):
    code = 0x2202
    data_fmt = ''

    def __init__(self, message):
        pass

    def execute(self):
        rospy.logdebug("Querying heartbeat pulse")
        return_message = ReportHeartbeatPulse()
        return return_message

    def verify_query_message(self, query_message):
        try:
            if len(query_message) == 0:
                return 0
            else:
                raise Exception("query_message is not of right length")
        except Exception as e:
            rospy.logerr(e.message)
        return 4


class ReportHeartbeatPulse(JausMessage):
    code = 0x4202
    data_fmt = ''

    def __init__(self):
        self.output = ""

    def execute(self):
        pass

