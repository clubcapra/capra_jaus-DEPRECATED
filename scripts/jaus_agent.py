#!/usr/bin/env python

import socket
import rospy
from std_msgs.msg import String, UInt8, UInt16, UInt32
from capra_jaus.msg import Service
from JUDPMessage import JUDPMessage
from messages.jaus_enums import *
from messages.QueryIdentification import QueryIdentification
from messages.QueryServices import QueryServices
from messages.QueryEvents import QueryEvents
from messages.CreateEvent import CreateEvent
from messages.UpdateEvent import UpdateEvent
from messages.CancelEvent import CancelEvent
from messages.QueryStatus import QueryStatus
from messages.QueryHeartbeatPulse import QueryHeartbeatPulse
from messages.QueryConfiguration import QueryConfiguration
from messages.QuerySubsystemList import QuerySubsystemList
from messages.RegisterServices import RegisterServices
from messages.TriggerEvent import TriggerEvent, ExecuteEvent
from messages.QueryVelocityState import QueryVelocityState
from messages.QueryLocalPose import QueryLocalPose
from messages.Standby import Standby
from messages.Resume import Resume
from messages.Reset import Reset
from messages.Shutdown import Shutdown
from messages.SetEmergency import SetEmergency
from messages.ClearEmergency import ClearEmergency
from messages.QueryAuthority import QueryAuthority
from messages.QueryTimeout import QueryTimeout
from messages.QueryControl import QueryControl
from messages.SetAuthority import SetAuthority
from messages.RequestControl import RequestControl
from messages.ReleaseControl import ReleaseControl
from messages.ControlResult import RejectControl
from capra_jaus.srv import QueryStatus as QueryStatusService
from capra_jaus.srv import VerifyMessage, GetAddedServices, GetAddedServicesResponse
from jaus_access_control_service import RejectControlEnum
from messages.PublisherFactory import PublisherFactory


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
address = ''
port = 0
server_address = ''
server_port = 0
status = Status.INIT
emergency_stack = []
added_services = []

message_types = [QueryIdentification, SetEmergency, QueryServices, QueryEvents, CreateEvent, UpdateEvent, CancelEvent,
                 QueryHeartbeatPulse, QueryConfiguration, QueryStatus, Standby, Resume, Reset, Shutdown, ClearEmergency,
                 QuerySubsystemList, RegisterServices, QueryVelocityState, QueryLocalPose, QueryAuthority, QueryTimeout,
                 QueryControl, SetAuthority, RequestControl, ReleaseControl]


def listener():
    global sock, server_address, server_port, address, port, added_services
    added_services = []
    rospy.Subscriber("/jaus_agent/execute", String, execute)
    rospy.Subscriber("/jaus_agent/execute_reject_controller", UInt32, executeRejectController)
    rospy.Subscriber("/jaus_agent/execute_event", String, execute_event)
    rospy.Subscriber("/jaus_agent/status_update", UInt8, status_update)
    rospy.Subscriber("/jaus_agent/emergency", UInt16, emergency)
    rospy.Subscriber("/jaus_agent/add_service", Service, add_service)
    rospy.Service('jaus_query_status', QueryStatusService, get_status)
    rospy.Service('jaus_verify_message', VerifyMessage, verify_message)
    rospy.Service('jaus_get_added_services', GetAddedServices, get_added_services)
    rospy.init_node("jaus_agent", log_level=rospy.DEBUG)

    address = rospy.get_param("~jaus_agent_address", '127.0.0.1')
    port = rospy.get_param("~jaus_agent_port", 3794)
    rospy.logdebug("Binding to %s:%s", address, port)
    sock.bind((address, port))
    server_address = rospy.get_param('~judge_server_address', '127.0.0.1')
    server_port = rospy.get_param('~judge_server_port', 3794)
    # Fix because for some reason the first to messages of status_update are ignored...
    publisher_su = PublisherFactory.get_publisher("/jaus_agent/status_update", UInt8, 10)
    s_su = UInt8()
    s_su.data = Status.INIT
    publisher_su.publish(s_su)
    publisher_su.publish(s_su)
    publisher_e = PublisherFactory.get_publisher("/jaus_agent/emergency", UInt16, 10)
    s_e = UInt16()
    s_e.data = EmergencyCode.STOP
    publisher_e.publish(s_e)
    publisher_as = PublisherFactory.get_publisher("/jaus_agent/add_service", Service, 10)
    serv = Service()
    serv.uri = ''
    serv.major_version_number = 0
    serv.minor_version_number = 0
    publisher_as.publish(serv)
    #^^^ Fix because for some reason the first to messages of status_update are ignored...
    rospy.spin()


def get_return_message(input_judp_message):
    for message_type in message_types:
        if message_type.code == input_judp_message.message_code:
            return message_type(input_judp_message)
    return None

def execute(data):
    global sock
    global server_address
    global server_port
    rospy.logdebug("jaus_agent received %s", data)
    input_judp_message = JUDPMessage()
    input_judp_message.parse(data.data)
    rospy.logdebug("%s", input_judp_message)

    jaus_message = get_return_message(input_judp_message)

    if jaus_message is not None:
        return_message = jaus_message.execute()
        if return_message is not None:
            return_message.execute()
            return_judp_message = JUDPMessage()
            return_judp_message.source_id = input_judp_message.destination_id
            return_judp_message.destination_id = input_judp_message.source_id
            return_judp_message.pack(return_message.code, return_message.output)
            rospy.logdebug("Sending response to %s:%s./r/nData: %s", server_address, server_port, ":".join("{:02x}".format(ord(c)) for c in return_judp_message.output))
            sock.sendto(return_judp_message.output, (server_address, server_port))
    else:
        rospy.logerr("There was a problem processing a message of type %s", hex(input_judp_message.message_code))

def executeRejectController(controller_id):
    global sock
    global server_address
    global server_port

    return_message = RejectControl(RejectControlEnum.CONTROL_RELEASED)
    return_message.execute()
    return_judp_message = JUDPMessage()
    #TODO set source id
    return_judp_message.destination_id = controller_id
    return_judp_message.pack(return_message.code, return_message.output)
    rospy.logdebug("Sending response to %s:%s./r/nData: %s", server_address, server_port, ":".join("{:02x}".format(ord(c)) for c in return_judp_message.output))
    sock.sendto(return_judp_message.output, (server_address, server_port))


def execute_event(data):
    global sock
    global server_address
    global server_port
    rospy.logdebug("jaus_agent_execute_event received %s", ":".join("{:02x}".format(ord(c)) for c in data.data))
    input_judp_message = JUDPMessage()
    input_judp_message.parse(data.data)
    rospy.logdebug("%s", input_judp_message)
    trigger_event = TriggerEvent(input_judp_message)
    event_id, sequence_number, query_message = trigger_event.execute()
    fake_judp_message = JUDPMessage()
    fake_judp_message.message_code = input_judp_message.message_code
    fake_judp_message.payload = query_message
    rospy.logdebug("Fake JUDP message: %s", fake_judp_message)
    query_message = get_return_message(fake_judp_message)
    return_message = query_message.execute()
    if return_message is not None:
        return_message = ExecuteEvent(event_id, sequence_number, return_message)
        return_message.execute()
        return_judp_message = JUDPMessage()
        return_judp_message.source_id = input_judp_message.destination_id
        return_judp_message.destination_id = input_judp_message.source_id
        return_judp_message.pack(return_message.code, return_message.output)
        rospy.logdebug("Sending response to %s:%s./r/nData: %s", server_address, server_port, ":".join("{:02x}".format(ord(c)) for c in return_judp_message.output))
        sock.sendto(return_judp_message.output, (server_address, server_port))
    else:
        rospy.logerr("Couldn't create a ExecuteEvent message.")


def get_status(data):
    global status
    return status


def status_update(new_status):
    global status
    rospy.logdebug('Updating status %s to %s', status, new_status.data)
    status = new_status.data


def verify_message(data):
    message_code = data.code
    query_message = data.query_message
    if is_supported(message_code):
        for message_type in message_types:
            if message_type.code == message_code:
                message = JUDPMessage()
                message.payload = query_message
                rospy.logdebug("Check if %s is ok with JUDPMessage %s", message_type, message)
                instance = message_type(message)
                return instance.verify_query_message(query_message)
    return 5


def is_supported(message_code):
    for message_type in message_types:
        if message_type.code == message_code:
            return True
    return False


def get_added_services(data):
    global added_services
    rospy.logdebug("Fetching added services: %s", added_services)
    services = GetAddedServicesResponse(added_services)
    return services


def add_service(service):
    global added_services
    rospy.logdebug("Registering service: %s", service)
    added_services.append(service)
    rospy.logdebug("added services is now: %s", added_services)


def emergency(emergency):
    global emergency_stack
    rospy.logdebug('New emergency %s', emergency.data)
    if emergency.data == EmergencyCode.NO_CODE:
        rospy.logdebug('Cleaning the emergency stack')
        emergency_stack = []
    else:
        rospy.logdebug('Adding new emergency %s', emergency.data)
        emergency_stack.append(emergency.data)


if __name__ == '__main__':
    listener()
