# Sonardyne USBL

This package interfaces with Sonardyne equipment to provide a ROS interface. Nodes are included to communicate with the Ranger software using XML based protocol of the Remote Control interface as well as to communicate directly with modems using the serial protocol over serial, TCP or UDP connections.

## modem_node

The `modem_node.py` node provides topics for sending and receiving SMS messages as well as sending and receiving raw modem commands. Diagnostics are also published over the standard `/diagnostics` topic.

Raw data coming from the modem is published as `std_msgs/String` messages on the `~raw` topic and SMS messages are published as `sonardyne_msgs/SMS` messages on the `~received_sms` topic.

Raw commands can be sent to the modem by publishing `std_msgs/String` messages to `~send_raw` while SMS messages can be sent as `sonardyne_msgs/SMS` messages to `~send_sms`.

### Parameters

The connection type is specified with the `~connection/type` parameter with expected values of `serial`, `tcp` or `udp`.

For a `serial` connection, the serial port is specified by the `~connection/port` parameter and the baud rate by `~connection/baud_rate`.

For a `tcp` connection, the host and port are specifed by `~connection/host` and `~connection/port`.

For a `udp` connection, the host, input port, and output port are specified by `~connection/host`, `~connection/input_port` and `~connection/output_port`.

## ranger_node

The `ranger_node.py` node subscribes to asynchronous position updates from the Ranger software and publishes them as `geographic_msgs/GeoPointStamped` messages. Devices that are tracked have their positions published to a topic under `~positions/` consiting of the device name.

The node also publishes position data as received as `sonardyne_msgs/Position` messages on the `~geographic_positions` topic as well as device status updates as `sonardyne_msgs/DeviceStatus` messages on the `~device_status` topic.

Active tracking of devices can be enabled or disabled by sending a `sonardune_msgs/DeviceEnable` message to the `~enable_tracking` topic. To find a devices UID, the `~device_status` topic can be monitored.

For monitoring and troubleshooting, the raw XML data received from the Ranger software is published on the `~raw_control` topic.

### Parameters

The following parameters specify the connection infromation.

#### ~host

The host running the Ranger software.

#### ~control_port_in and ~control_port_out

The UDP ports configured in the Ranger software for the Remote Control interface.
