import rospy
import struct
from jaus_message import JausMessage
from capra_jaus.msg import Service
from PublisherFactory import PublisherFactory


class RegisterServices(JausMessage):
    code = 0x0b00
    data_fmt = '<B'
    service_list_size_index = 0

    def __init__(self, message):
        self.parse(message.payload[0])
        self.message = message

    def execute(self):
        list_size = self.data[self.service_list_size_index]
        stripped_data = self.message.payload[1::]
        publisher = PublisherFactory.get_publisher('/jaus_agent/add_service', Service, 10)
        rospy.logdebug("Registering %s services", list_size)
        for i in xrange(list_size):
            service_name_length = struct.unpack('<B', stripped_data[0])[0]
            stripped_data = stripped_data[1::]
            service_name = stripped_data[0:service_name_length]
            stripped_data = stripped_data[service_name_length::]
            major_version, minor_version = struct.unpack('<BB', stripped_data[0:2])
            stripped_data = stripped_data[2::]
            service = Service()
            service.uri = service_name
            service.major_version_number = major_version
            service.minor_version_number = minor_version
            rospy.logdebug("Publishing the registration of service: %s", service)
            publisher.publish(service)
            rospy.logdebug("Publishing is over.")
        return None
