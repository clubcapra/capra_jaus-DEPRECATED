#!/usr/bin/env python

import rospy
import struct
from subprocess import call
from capra_jaus.srv import *
from capra_jaus.msg import Event
from capra_jaus.msg import ConfirmEventRequest
from capra_jaus.msg import RejectEventRequest
active_events = {}
next_id = 2
event_index = 0
rate_index = 1
node_index = 2

def run(cmd):
    rospy.loginfo("Running command:{0}".format(cmd))
    call(str(cmd) + " &", shell=True)

def stop_node(node_name):
    command = "rosnode kill " + node_name
    run(command)

def start_node(package, node_name, params = ""):
    command = "rosrun {0} {1} {2} _clear_params=true".format(package, node_name, params)
    run(command)

def listener():
    rospy.init_node("jaus_events_manager", log_level=rospy.DEBUG)
    rospy.Service('jaus_query_events', QueryEvents, query_events)
    rospy.Service('jaus_create_event', CreateEvent, create_event)
    rospy.Service('jaus_cancel_event', CancelEvent, cancel_event)
    rospy.Service('jaus_update_event', UpdateEvent, update_event)
    rospy.spin()

def query_events(data):
    global active_events
    events = []
    if data.event_id > 0:
        if data.event_id in active_events:
            event = active_events[data.event_id]
            events.append(event)
        else:
            rospy.logerr('Could not retrieve event of id:%s', data.event_id)
    elif data.message_id > 0:
        for key, event in active_events.iteritems():
            if event.query_message[0:2] == hex(data.message_id):
                events.append(event)
    elif data.event_type > 0:
        for key, event in active_events.iteritems():
            if event.event_type == data.event_type:
                events.append(event)
    else:
        for key, event in active_events.iteritems():
            events.append(event)
    return events

def create_event(data):
    global active_events, next_id
    if data.event_type == 1:
        validate_code = 2
    else:
        validate_code = validate_query_message(data.query_message)
    if validate_code == 0:
        event_id = next_id
        next_id += 1
        event = Event()
        event.event_type = data.event_type
        event.event_id = next_id
        event.query_message = data.query_message
        event.create_message = data.sender_info
        node_name = "jaus_event_%s" % (str(event_id))
        active_events[event_id] = (event, data.requested_rate, node_name)
        rospy.logdebug("Created event: %s", event)
        size = 21 + len(data.query_message)
        event_message = struct.pack("<BBHBII", data.sender_info.version, data.sender_info.type_hc, size,
                        data.sender_info.pbad_flags, data.sender_info.destination_id, data.sender_info.source_id)
        event_message += data.query_message[0:2]
        event_message += struct.pack("<BBI", event_id, 0, len(data.query_message))
        event_message += data.query_message[2::]
        event_message += struct.pack('<H', data.sender_info.sequence_number)

        message = event_message.encode("hex") + "h"
        start_node("capra_jaus", "jaus_event.py", "__name:=%s _jaus_event_rate:=%s _jaus_event_message:='%s'" % (node_name, data.requested_rate, message))
        confirm_message = ConfirmEventRequest()
        confirm_message.request_id = data.request_id
        confirm_message.event_id = next_id
        confirm_message.confirmed_rate = data.requested_rate
        return True, confirm_message, None
    else:
        reject_message = RejectEventRequest()
        reject_message.presence_vector = 0
        reject_message.request_id = data.request_id
        reject_message.response_code = validate_code
        return False, None, reject_message

def update_event(data):
    global active_events, event_index, node_index
    event_id = data.event_id
    if data.event_type == 1:
        rospy.logdebug("update_event check on event type failed because it's on change")
        validate_code = 2
    elif event_id not in active_events:
        rospy.logdebug("update_event check on event type failed because it's not an existing event")
        validate_code = 6
    else:
        rospy.logdebug("update_event check on event type failed because the query_message is not valid")
        validate_code = validate_query_message(data.query_message)
    if validate_code == 0:
        event = active_events[event_id][event_index]
        old_name = active_events[event_id][node_index]
        event.query_message = data.query_message
        event.create_message = data.sender_info
        node_name = old_name + "_" + (str(event_id))
        active_events[event_id] = (event, data.requested_rate, node_name)
        stop_node(old_name)

        rospy.logdebug("Created event: %s", event)
        size = 21 + len(data.query_message)
        event_message = struct.pack("<BBHBII", data.sender_info.version, data.sender_info.type_hc, size,
                        data.sender_info.pbad_flags, data.sender_info.destination_id, data.sender_info.source_id)
        event_message += data.query_message[0:2]
        event_message += struct.pack("<BBI", event_id, 0, len(data.query_message))
        event_message += data.query_message[2::]
        event_message += struct.pack('<H', data.sender_info.sequence_number)

        message = event_message.encode("hex") + "h"
        start_node("capra_jaus", "jaus_event.py", "__name:=%s _jaus_event_rate:=%s _jaus_event_message:='%s'" % (node_name, data.requested_rate, message))

        confirm_message = ConfirmEventRequest()
        confirm_message.request_id = data.request_id
        confirm_message.event_id = next_id
        confirm_message.confirmed_rate = data.requested_rate
        return True, confirm_message, None
    else:
        reject_message = RejectEventRequest()
        reject_message.presence_vector = 0
        reject_message.request_id = data.request_id
        reject_message.response_code = validate_code
        return False, None, reject_message

def cancel_event(data):
    global active_events, node_index
    rospy.logdebug("In cancel_event service")
    validate_code = 0
    if data.event_id not in active_events:
        rospy.logdebug("event_id: %s is not in events so we reject it", data.event_id)
        validate_code = 6
    if validate_code == 0:
        rate = active_events[data.event_id][rate_index]
        event_name = active_events[data.event_id][node_index]
        stop_node(event_name)
        del active_events[data.event_id]
        confirm_message = ConfirmEventRequest()
        confirm_message.request_id = data.request_id
        confirm_message.event_id = data.event_id
        confirm_message.confirmed_rate = rate
        return True, confirm_message, None
    else:
        reject_message = RejectEventRequest()
        reject_message.presence_vector = 0
        reject_message.response_code = validate_code
        reject_message.request_id = data.request_id
        return False, None, reject_message

def set_emergency(data):
    global active_events
    pass

def validate_query_message(query_message):
    code = struct.unpack('<H', query_message[0:2])[0]
    query_message = query_message[2::]
    rospy.wait_for_service('jaus_verify_message')
    verify_message_call = rospy.ServiceProxy('jaus_verify_message', VerifyMessage)
    rospy.logdebug("verify message call: code:%s, query_message:%s", code, ":".join("{:02x}".format(ord(c)) for c in query_message))
    result = verify_message_call(code, query_message)
    return result.valid_code

listener()
