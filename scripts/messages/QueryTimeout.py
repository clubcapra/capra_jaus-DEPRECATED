from jaus_message import JausMessage
import rospy
import struct


class QueryTimeout(JausMessage):
    code = 0x2003
    data_fmt = ''

    def __init__(self, message):
        pass

    def execute(self):
        #TODO get timeout
        timeout = 0
        return ReportTimeout(timeout)

    def verify_query_message(self, query_message):
        if len(query_message) == 0:
            return 0
        return 4


class ReportTimeout(JausMessage):
    code = 0x4003
    data_fmt = '<B'

    def __init__(self, timeout):
        self.timeout = timeout

    def execute(self):
        rospy.logdebug('Reporting timeout = %s', self.timeout)
        self.pack((self.timeout, ))

