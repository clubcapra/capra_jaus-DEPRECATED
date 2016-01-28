# JAUS Challenge
Pris de la version 2011 des règles de l'[IGVC](http://www.igvc.org/rules.html)
## TECHNICAL OVERVIEW
Each entry will interface with the Judge’s COP providing information as specified below. The general approach to the JAUS interface will be to respond to a periodic status and position requests from the COP. This requires the support of the JAUS Transport Specification (AS5669A) and the JAUS Core Service Set (AS5710). The JAUS Transport Specification supports several communication protocols, the competition will use only the Ethernet based JUDP. The Core services required for the competition include the discovery, access control, and management services. The JAUS Mobility Service Set (AS6009) or JSS-Mobility defines the messaging to be used for position communications and waypoint based navigation.
## COMMON OPERATING PICTURE
The COP will provide a high level view of the systems in operation that successfully implement the JAUS protocol as described above. This software is a simple validation, reporting and recording tool for the Judges to use while verifying student implementations of the JAUS standard. It provides a graphical display of the operational area in relative coordinates. Primitive graphics are loaded in the display of the COP to add perspective. Each reported status is displayed on the COP user interface and recorded for future reference. For competitions and systems reporting positional data, a 2-D map on the COP display is annotated with the updated position as well as track marks showing the previous position of the system for the current task.
## COMMUNICATIONS PROTOCOLS
The teams will implement a wireless 802.11b/g or hardwired Ethernet (RJ-45) data link. The interface can be implemented at any point in the student team’s system including the control station or mobility platform.

  The Internet Protocol (IP) address to be used will be provided at the competition. For planning purposes, this address will be in the range of 192.168.1.100 to 192.168.1.200. The Judge’s COP will have both hard-wire and 802.11b/g capabilities where the IP address of the COP will be 192.168.1.42.  All teams will be provided an IP address to be used during the competition. The last octet of the IP address is significant, as it will also be used as the subsystem identifier in the team’s JAUS ID. The port number for all JAUS traffic shall be 3794.
## JAUS SPECIFIC DATA
The JAUS ID mentioned above is a critical piece of data used by a JAUS node to route messages to the correct process or attached device. As indicated above each team will be provided an IP address in which the last octet will be used in their respective JAUS ID. A JAUS ID consists of three elements, a Subsystem ID, a Node ID and a Component ID. The Subsystem ID uniquely identifies a major element that is an unmanned system, an unmanned system controller or some other entity on a network with unmanned systems. A Node ID is unique within a subsystem and identifies a processing element on which JAUS Components can be found. A Component ID is unique within a Node represents an end-point to and from which JAUS messages are sent and received. The last octet of the assigned IP address will be used as the team’s JAUS Subsystem ID. So for the team assigned the IP address of 192.168.1.155, the completed JAUS ID of the position-reporting component might be 155-1-1 where the node and component are both assigned the IDs of 1. This is shown in the IP and JAUS ID Assignment Figure below. The Node ID and Component ID are discussed further in the JAUS Service Interface

  Definition Language standard (AS5684). The COP software will be programmed with the assumption that all services required by the specific competition are implemented on a single component.

![JausId](http://www.igvc.org/rules_files/image010.jpg)
*IP and JAUS ID Assignment*

  In summary, each team will be assigned an IP address by the judges. The last octet of that IP address will be the team’s subsystem identifier. The COP will be a subsystem as will each team’s entry in the competition. The COP will have a JAUS ID of 42:1:1 and an IP address of 192.168.1.42. The port number shall be 3794.

## COMPETITION TASK DESCRIPTION

Messages passed between the COP and the team entries will include data as described in the task descriptions below. The COP will initiate all requests subsequent to the discovery process described as Task 1. A system management component is required of all teams. This interface will implement several of the messages defined by the Management Service defined in the JSS-Core. This service inherits the Access Control, Events and Transport services also defined by the JSS-Core document. The implementation of the Access Control interfaces will be necessary to meet the JAUS Challenge requirements; however no messages from the Events service will be exercised. The sequence diagram in Discovery and System Management Figure shows the required transactions for discovery including the access control setup and system control protocol. This interaction is required for every task.

  The judges will evaluate each team’s ability to meet the Interoperability Challenge for the tasks described below in accordance with the scoring chart.

|Judges will score the task as follows:|Maximum Points|
| -------------------------------------|:------------:|
|**1.** Transport Discovery|10|
|**2.** Capabilities Discovery|10|
|**3.** System Management|10|
|**4.** Velocity State Report|10|
|**5.** Position and Orientation Report|10|
|**6.** Waypoint Navigation|10|
|**Total**|**60**|


## TRANSPORT DISCOVERY

For any two elements in the system to communicate meaningful data there must first be a handshake to ensure both sides use the same protocols and are willing participants in the interaction.  For the sake of simplicity, the team’s entry shall initiate the discovery protocol with the Judge’s COP, and the IP address and JAUS ID of the COP shall be fixed.  The IP address and JAUS ID of the Judge’s COP are defined as:


|COP IP ADDRESS|COP JAUS ID|
|192.168.1.42:3794|42-1-1 (Subsystem-Node-Component)|

The discovery process, in Discovery and System Management Figure, will occur at the application layer. The student team’s JAUS element will send a request for identification to the COP once every 5 seconds. The COP will respond with the appropriate informative message and request identification in return from the team’s JAUS interface. After the identification report from the COP, the team entry will stop repeating the request. This transaction will serve as the basic discovery between the two elements.


  The COP software will be programmed with the assumption that all services required by the specific competition are provided at the single JAUS ID. Furthermore, as per the AS5669A Specification, the team’s entry shall receive JUDP traffic at the same IP address and port number that initiated the discovery protocol. Teams should note that this is different from common UDP programming approaches in which the outbound port for sent messages is not bound.

![DSM](http://www.igvc.org/rules_files/image012.jpg)
*Discovery and System Management*

The following table shows the messages sent from the COP to the team’s entry, along with the expected response and minimal required fields to be set using the presence vector (PV) if applicable, required to complete this portion of the challenge



|Input Messages|Expected Response|Required Fields (PV)|
|----|----|----|
|Query Identification|Report Identification|N/A|



## CAPABILITIES DISCOVERY
Following the completion of the Transport Discovery handshake the COP will query the entry for its capabilities. The Query Services message and Report Services message are defined in the AS5710 document and require the inheritance of the Transport service. The COP will send a Query Services message to a student team entry. Upon receipt of the message the student team entry shall respond with a properly formed Report Services message.

  The following table shows the messages sent from the COP to the team’s entry, along with the expected response and minimal required fields to be set using the presence vector (PV) if applicable, required to complete this portion of the challenge:
|Input Messages|Expected Response|Required Fields (PV)|
|----|----|----|
|Query Identification|Report Identification|N/A|


## SYSTEM MANAGEMENT
The implementation of the status report is required. This interoperability task, like the discovery tasks above, is also a prerequisite for all other tasks. The task begins with the discovery handshake as described above and continues for an indeterminate period of time. The protocol is given in Discovery and System Management Figure. The following table shows the messages sent from the COP to the team’s entry, along with the expected response and minimal required fields to be set using the presence vector (PV) if applicable, required to complete this portion of the challenge:


|Input Messages|Expected Response|Required Fields (PV)|
|----|----|----|
|Query Control|Report Control|N/A|
|Request Control|Confirm Control|N/A|
|Query Status|Report Status|N/A|
|Resume|<none>|N/A|
|Standby|<none>|N/A|
|Shutdown|<none>|N/A|


## VELOCITY STATE REPORT
In the Velocity State Report task the COP will query the entry for its current velocity state. The COP will send a Query Velocity State message to a student team entry. Upon receipt of the message the student team entry shall respond with a properly formed Report Velocity State message.


  The following table shows the messages sent from the COP to the team’s entry, along with the expected response and minimal required fields to be set using the presence vector (PV) if applicable, required to complete this portion of the challenge:

|Input Messages|Expected Response|Required Fields (PV)|
|----|----|----|
Query Velocity State|Report Velocity State|Velocity X, Yaw Rate & Time Stamp (320 Decimal, 0140h)|


## POSITION AND ORIENTATION REPORT
For performing the task Position and Orientation Report, the discovery and status protocols described above are also required.  In addition to the COP queries for status, the vehicle systems will also be required to respond correctly to local position queries.  The reports will be validated for relative position and with respect to a relative time offset to ensure the time contained within each position report is valid with respect to some timer within the entry’s system. In other words, the position reports must show that the travel occurred at a reasonable speed and not instantaneously.  Additional variation in the position reporting using the available presence vectors is allowed.  Minimally, all entries must report X, Y and Time Stamp.

The following table shows the messages sent from the COP to the team’s entry, along with the expected response and minimal required fields to be set using the presence vector (PV) if applicable, required to complete this portion of the challenge:

|Input Messages|Expected Response|Required Fields (PV)|
|----|----|----|
|Set Local Pose|<none>|X, Y & Yaw (67 Decimal, 0043h)|
|Query Local Pose|Report Local Pose|X, Y & Time Stamp (259 Decimal, 0103h)|


## WAYPOINT NAVIGATION
The team entry shall implement the Local Waypoint List Driver service from the JAUS Mobility Service Set (AS6009). From a starting point in the JAUS challenge test area the student entry will be commanded to traverse, in order, a series of 4 waypoints. Time will be kept and will start at the moment that the student entry exits the designated start box. Upon leaving the start box the student entry will proceed to the first waypoint in the list. Upon satisfactorily achieving each waypoint the team will be credited with 2.5 points. Time is kept for each waypoint achieved. The shortest overall time taken to achieve this task will determine the winner in the event of a tie.

The following table shows the messages sent from the COP to the team’s entry, along with the expected response and minimal required fields to be set using the presence vector (PV) if applicable, required to complete this portion of the challenge:


|Input Messages|Expected Response|Required Fields (PV)|
|----|----|----|
|Set Element|Confirm Element Request|N/A|
|Query Element List|Report Element List|N/A|
|Query Element Count|Report Element Count|N/A|
|Execute List|<none>|N/Speed (value of 1)|
|Query Active Element|Report Active Element|N/A|
|Query Travel|Report Travel Speed|N/A|
|Query Local Waypoint|Report Local Waypoint|X & Y (value of 3)|


