#!/usr/bin/env python

import rospy
from capra_jaus.msg import SetAuthority
from capra_jaus.srv import QueryAuthority
from capra_jaus.srv import QueryControl
from capra_jaus.srv import RequestControl
from capra_jaus.srv import ReleaseControl
from capra_jaus.srv import IsControllingClient
from std_msgs.msg import Bool, UInt32


class AccessControlService:

    def __init__(self):
        rospy.init_node('jaus_access_control', log_level=rospy.DEBUG)
        default_authority = rospy.get_param("~default_authority", 0)
        self.defaultAuthority = default_authority
        self.currentAuthority = default_authority
        self.isControlled = False
        self.isControlAvailable = True
        self.currentSourceId = 0
        self.timeout = 0

        self.publisher = rospy.Publisher('/jaus_agent/execute_reject_controller', UInt32, queue_size=10)

        rospy.Service('jaus_query_authority', QueryAuthority, self.get_authority)
        rospy.Service('jaus_query_control', QueryControl, self.get_control)
        rospy.Service('jaus_request_control', RequestControl, self.requestControl)
        rospy.Service('jaus_release_control', ReleaseControl, self.releaseControl)
        rospy.Service('jaus_is_controlling_client', IsControllingClient, self.isControllingClientService)
        rospy.Subscriber("/jaus_access_control/set_authority", SetAuthority, self.setAuthority)
        rospy.Subscriber('/jaus_access_control/set_is_control_available', Bool, self.setIsControlAvailable)

        rospy.spin()

    def isDefaultAuthorityGreater(self, authority):
        return self.defaultAuthority > authority

    def isCurrentAuthorityLess(self, authority):
        return self.currentAuthority < authority

    def isAuthorityValid(self, authority):
        return authority >= self.defaultAuthority and \
               authority <= self.currentAuthority

    def isControllingClientService(self, data):
        rospy.logdebug('self.currentSourceId = %d ||| source_id = %d', self.currentSourceId, data.source_id)
        return self.currentSourceId == data.source_id

    def isControllingClient(self, source_id):
        rospy.logdebug('self.currentSourceId = %d ||| source_id = %d', self.currentSourceId, source_id)
        return self.currentSourceId == source_id

    def get_authority(self, data):
        return self.currentAuthority

    def get_control(self, data):
        subsystem_id = (self.currentSourceId >> 16) & 0xff
        node_id = (self.currentSourceId >> 8) & 0xff
        component_id = self.currentSourceId & 0xff
        rospy.logdebug('ReportControl Subsystem_id = %s ||| Node_id = %s ||| Component_id = %s ||| Authority = %s',
                       subsystem_id, node_id, component_id, self.currentAuthority)
        return (subsystem_id, node_id, component_id, self.currentAuthority)

    def storeID(self,id):
        self.currentSourceId = id

    def setAuthority(self, authorityMsg):
        if self.isControllingClient(authorityMsg.source_id) and self.isAuthorityValid(authorityMsg.authority_code) \
                and self.isControlAvailable:
            rospy.logdebug('Setting Authority by source_id = %s with authority = %s', authorityMsg.source_id, authorityMsg.authority_code)
            self.currentAuthority = authorityMsg.authority_code
        else:
            rospy.logdebug("Can't set authority, ")

    def setIsControlAvailable(self, isControlAvailable):
        self.isControlAvailable = isControlAvailable

    def requestControl(self, data):
        authority_code = data.authority_code
        source_id = data.source_id
        response_code = ConfirmControlEnum.NOT_AVAILABLE #A1, C1
        if self.isControlAvailable:
            if self.isDefaultAuthorityGreater(authority_code):
                response_code = ConfirmControlEnum.INSUFFICIENT_AUTHORITY #A2
                if self.isControllingClient(source_id):
                    self.sendRejectControlToController(RejectControlEnum.CONTROL_RELEASED) #D2
            elif not self.isControlled:
                self.storeID(source_id)
                self.currentAuthority = authority_code
                self.resetTimer()
                self.isControlled = True
                response_code = ConfirmControlEnum.CONTROL_ACCEPTED #B
            elif self.isControllingClient(source_id):
                self.resetTimer()
                self.currentAuthority = authority_code
                response_code = ConfirmControlEnum.CONTROL_ACCEPTED #C4
            elif not self.isCurrentAuthorityLess(authority_code):
                response_code = ConfirmControlEnum.INSUFFICIENT_AUTHORITY #C3
            else:
                self.sendRejectControlToController(RejectControlEnum.CONTROL_RELEASED)
                self.storeID(source_id)
                self.currentAuthority = authority_code
                response_code = ConfirmControlEnum.CONTROL_ACCEPTED #C2

        rospy.logdebug('Control requested by source_id = %s with authority = %s ||| response_code = %s', source_id, authority_code, response_code)
        return response_code

    def releaseControl(self, data):
        response_code = RejectControlEnum.NOT_AVAILABLE
        if self.isControlAvailable and (self.isControllingClient(data.source_id) or not self.isControlled):
            response_code = RejectControlEnum.CONTROL_RELEASED
            self.isControlled = False

        rospy.logdebug('Control released by source_id = %s ||| response_code = %s', data.source_id, response_code)
        return response_code

    def resetTimer(self):
        rospy.logdebug('Resetting timer')
        #TODO reset timer
        pass

    def sendRejectControlToController(self, responseCode):
        rospy.logdebug('Rejecting current controller = %s', self.currentSourceId)
        #TODO add response code to publisher
        self.publisher.publish(self.currentSourceId)

class ConfirmControlEnum():
    CONTROL_ACCEPTED = 0
    NOT_AVAILABLE = 1
    INSUFFICIENT_AUTHORITY = 2

class RejectControlEnum():
    CONTROL_RELEASED = 0
    NOT_AVAILABLE = 1

if __name__ == '__main__':
    AccessControlService()
