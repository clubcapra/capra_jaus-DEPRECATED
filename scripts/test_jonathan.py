import socket
import sys

client_ip = '192.168.0.123' #Robot
server_ip = '192.168.0.110' #Running that script

has_response = True
if len(sys.argv) > 2:
    client_ip = sys.argv[1]
    msg_name = sys.argv[2]
    if msg_name == 'QueryStatus':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x02\x20\xe0\x09'
    elif msg_name == 'Standby':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x03\x00\xe0\x09'
        has_response = False
    elif msg_name == 'Resume':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x04\x00\xe0\x09'
        has_response = False
    elif msg_name == 'Reset':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x05\x00\xe0\x09'
        has_response = False
    elif msg_name == 'Shutdown':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x02\x00\xe0\x09'
        has_response = False
    elif msg_name == 'SetEmergency':
        message = '\x02\x00\x12\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x06\x00\x01\x00\xe0\x09'
        has_response = False
    elif msg_name == 'ClearEmergency':
        message = '\x02\x00\x12\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x07\x00\x01\x00\xe0\x09'
        has_response = False
    elif msg_name == 'RequestControl':
        message = '\x02\x00\x11\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x0d\x00\x05\xe0\x09' #request control with authority 5
        has_response = True
    elif msg_name == 'ReleaseControl':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x0e\x00\xe0\x09'
        has_response = True
    elif msg_name == 'QueryControl':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x0d\x20\xe0\x09'
        has_response = True
    elif msg_name == 'QueryAuthority':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x01\x20\xe0\x09'
        has_response = True
    elif msg_name == 'QueryTimeout':
        message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x03\x20\xe0\x09'
        has_response = True
    elif msg_name == 'SetAuthority':
        message = '\x02\x00\x11\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x01\x00\x07\xe0\x09' #setting authority to 7
        has_response = False

print 'client_ip', client_ip
print 'server_ip', server_ip
print 'msg_name', msg_name
print 'message', message

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((server_ip, 3794))

robot_address = (client_ip, 3794)

# 0: Version
# 1: type_hc
# 2: size
# 3: size
# 4: pbad_flags
# 5: destination component id
# 6: destination node id
# 7: destination subsystem id
# 8: destination system id
# 9: source component id
# 10: source node id
# 11: source subsystem id
# 12: source component id
# 13: message_type
# 14: message_type
# 15: data
# ...: data
# -2: sequence_number
# -1: sequence_number


Node_cout = 1
node_id = 1
component_count = 2
component_id = 1
component_id = 2

try:

    # Send data
    print 'sending "%s"' % message

    print 'sending %s' % ":".join("{:02x}".format(ord(c)) for c in message)
    sent = sock.sendto(message, robot_address)
    
    if has_response:
        # Receive response
        print 'waiting to receive'
        data, server = sock.recvfrom(4096)
        print 'received "%s"' % data
        print 'received %s' % ":".join("{:02x}".format(ord(c)) for c in data)

finally:
    print 'closing socket'
    sock.close()
