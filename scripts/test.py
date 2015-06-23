import socket

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(('192.168.0.110', 3794)) #  192.168.0.110

server_address = ('192.168.0.123', 3794)  # ('192.168.0.123', 3794)  # ('239.255.0.1', 3794)
#message = '\x02\x00\x11\x00\x01\x01\x01\x6d\x00\x01\x01\x2a\x00\x0d\x24\x00\xd5\x09'
#message = '\x02\x00\x11\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x00\x2b\x02\xd6\x09'  # Query Identification
#message = '\x02\x00\x14\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x03\x2b\x01\xff\x01\xff\xd7\x09'  # Query Services
#message = '\x02\x00\x12\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\xf0\x21\x01\x00\xd8\x09'  # Query Events
message = '\x02\x00\x1b\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\xf0\x01\xd3\x00\x01\x00\x03\x00\x00\x00\x00\x2b\x02\xd9\x09'  # CreateEvent
#message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x02\x20\xe0\x09' # Query Status 2002
#message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x03\x00\xd7\x09'  # Standby 3
#message = '\x02\x00\x10\x00\x01\xff\xff\xff\xff\x01\x01\x2a\x00\x04\x00\xd7\x09'  # Resume 4

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
    sent = sock.sendto(message, server_address)

    # Receive response
    print 'waiting to receive'
    data, server = sock.recvfrom(4096)
    print 'received "%s"' % data
    print 'received %s' % ":".join("{:02x}".format(ord(c)) for c in data)

finally:
    print 'closing socket'
    sock.close()
