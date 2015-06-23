from jaus_message import JausMessage
import rospy
import struct
from capra_jaus.srv import QueryAuthority as QueryAuthorityService


class QueryAuthority(JausMessage):
    code = 0x2001
    data_fmt = ''

    def __init__(self, message):
        pass

    def execute(self):
        rospy.wait_for_service('jaus_query_authority')
        query_authority_call = rospy.ServiceProxy('jaus_query_authority', QueryAuthorityService)
        query_authority_result = query_authority_call()
        rospy.logdebug('JAUS current authority is %d', query_authority_result.authority_code)
        return ReportAuthority(query_authority_result.authority_code)

class ReportAuthority(JausMessage):
    code = 0x4001
    data_fmt = '<B'

    def __init__(self, authority):
        self.authority = authority

    def execute(self):
        self.pack((self.authority, ))


