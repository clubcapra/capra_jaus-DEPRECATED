import rospy
import struct
from capra_jaus.srv import GetAddedServices
from jaus_message import JausMessage


class QueryServices(JausMessage):
    code = 0x2b03
    data_fmt = '<B'
    node_list_size_index = 0

    def __init__(self, message):
        self.parse(message.payload[0])
        self.component_id = message.destination_id & 0xFF
        self.message = message
        self.to_report = {}

    def execute(self):
        list_size = self.data[self.node_list_size_index]
        stripped_data = self.message.payload[1::]
        for i in xrange(list_size):
            node_id, components_count = struct.unpack('<BB', stripped_data[0:2])
            stripped_data = stripped_data[2::]
            components = []
            for j in xrange(components_count):
                component_id = struct.unpack('<B', stripped_data[0 + j:0 + j + 1])[0]
                components.append(component_id)
            stripped_data = stripped_data[components_count::]
            self.to_report[node_id] = components

        return ReportServices(self.to_report)

    def verify_query_message(self, query_message):
        try:
            self.parse(query_message[0])
            list_size = self.data[self.node_list_size_index]
            stripped_data = query_message[1::]
            for i in xrange(list_size):
                node_id, components_count = struct.unpack('<BB', stripped_data[0:2])
                stripped_data = stripped_data[2::]
                components = []
                for j in xrange(components_count):
                    component_id = struct.unpack('<B', stripped_data[0 + j:0 + j + 1])[0]
                    components.append(component_id)
                stripped_data = stripped_data[components_count::]
                self.to_report[node_id] = components
            return 0
        except Exception as e:
            rospy.logerr(e.message)
        return 4

class ReportServices(JausMessage):
    code = 0x4b03
    data_fmt = '<B'

    def __init__(self, nodes_to_report):
        rospy.logdebug("Nodes to report: %s", nodes_to_report)
        self.nodes_to_report = nodes_to_report

    def execute(self):
        nodes_count = len(self.nodes_to_report)  # S'il y en a 1, on est suppose mettre le node Id, qui est 1
        self.output = struct.pack("<B", nodes_count)
        for node_id, components in self.nodes_to_report.iteritems():
            if node_id == 255:
                self.output += struct.pack("<B", 1)
            else:
                self.output += struct.pack("<B", node_id)
            self.output += struct.pack("<B", len(components))
            for component_id in components:
                rospy.logdebug("Component_id: %s", component_id)
                self.output += struct.pack("<BB", component_id, 0)
                if component_id == 1 or component_id == 255:  # Platform Management
                    rospy.wait_for_service('jaus_get_added_services')
                    get_added_services_call = rospy.ServiceProxy('jaus_get_added_services', GetAddedServices)
                    added_services = get_added_services_call()
                    rospy.logdebug("Fetched %s added services", len(added_services.services))
                    self.output += struct.pack("<B", 11 + len(added_services.services))
                    #Transport, urn:jaus:jss:core:Transport, v1.0
                    name = "urn:jaus:jss:core:Transport"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Events, urn:jaus:jss:core:Events, v1.0
                    name = "urn:jaus:jss:core:Events"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Access Control, urn:jaus:jss:core:Access Control, v1.0
                    name = "urn:jaus:jss:core:AccessControl"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Liveness, urn:jaus:jss:core:Liveness, v1.0
                    name = "urn:jaus:jss:core:Liveness"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Discovery, urn:jaus:jss:core:Discovery, v1.0
                    name = "urn:jaus:jss:core:Discovery"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    # Reporting and Navigation
                    #Management, urn:jaus:jss:core:Management, v1.0
                    name = "urn:jaus:jss:core:Management"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Waypoint Driver, urn:jaus:jss:mobility:LocalWaypointDriver, v1.0
                    name = "urn:jaus:jss:mobility:LocalWaypointDriver"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Waypoint List Driver, urn:jaus:jss:mobility:LocalWaypointListDriver, v1.0
                    name = "urn:jaus:jss:mobility:LocalWaypointListDriver"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Velocity State Sensor, urn:jaus:jss:mobility:VelocityStateSensor, v1.0
                    name = "urn:jaus:jss:mobility:VelocityStateSensor"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Local Pose Sensor, urn:jaus:jss:mobility:LocalPoseSensor, v1.0
                    name = "urn:jaus:jss:mobility:LocalPoseSensor"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    #Primitive Driver, urn:jaus:jss:mobility:PrimitiveDriver, v1.0
                    name = "urn:jaus:jss:mobility:PrimitiveDriver"
                    self.output += struct.pack("<B", len(name))
                    self.output += name
                    self.output += struct.pack("<BB", 1, 0)
                    for service in added_services.services:
                        self.output += struct.pack("<B", len(service.uri))
                        self.output += service.uri
                        self.output += struct.pack("<BB", service.major_version_number, service.minor_version_number)
                else:
                    rospy.logerr("Component Id %s does not exist", component_id)
