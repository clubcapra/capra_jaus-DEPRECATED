import rospy
from jaus_message import JausMessage
from std_msgs.msg import UInt8
from jaus_enums import Status
from capra_jaus.srv import QueryStatus, IsControllingClient
from PublisherFactory import PublisherFactory
from ReleaseControl import ReleaseControl


class Reset(JausMessage):
    # as5710 p40
    code = 0x5

    def __init__(self, message):
        self.message = message
        self.source_id = message.source_id
        self.publisher = PublisherFactory.get_publisher("/jaus_agent/status_update", UInt8, 10)

    def execute(self):
        rospy.logdebug('Receiving command RESET from %s', self.source_id)
        rospy.wait_for_service('jaus_query_status')
        query_status_call = rospy.ServiceProxy('jaus_query_status', QueryStatus)
        status = query_status_call().status
        rospy.logdebug('JAUS current status is %d', status)
        rospy.wait_for_service('jaus_is_controlling_client')
        controlling_status_call = rospy.ServiceProxy('jaus_is_controlling_client', IsControllingClient)
        is_controlling_client = controlling_status_call(self.source_id).is_controlling_client
        rospy.logdebug('JAUS current is_controlling_client state is %s', is_controlling_client)

        if is_controlling_client:
            s = UInt8()
            s.data = Status.STANDBY
            rospy.loginfo('Publishing the command RESET')
            self.publisher.publish(s)
            rc = ReleaseControl(self.message)
            rc.execute()
        else:
            rospy.logwarn("The client isn't controlling the robot, the command RESET will be ignored.")

