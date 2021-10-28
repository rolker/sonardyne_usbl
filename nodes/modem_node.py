#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

import sonardyne_usbl.modem

message_pub = None
raw_pub = None
modem = None

def timerCallback(event):
    data = modem.getResponse()
    if data is not None:
        print (data)

if __name__ == '__main__':
    rospy.init_node('sonardyne_modem')
  
    connection_type = rospy.get_param('~connection/type')
    if connection_type == 'serial':
        port = rospy.get_param('~connection/port')
        baud_rate = rospy.get_param('~connection/baud_rate', 9600)
        c = sonardyne_usbl.modem.SerialConnection(port, baud_rate)
        modem = sonardyne_usbl.modem.Modem(c)
    elif connection_type == 'tcp':
        host = rospy.get_param('~connection/host')
        port = rospy.get_param('~connection/port', 4000)
        c = sonardyne_usbl.modem.SerialConnection(host, port)
        modem = sonardyne_usbl.modem.Modem(c)

    message_pub = rospy.Publisher('messages', String, queue_size=10)
    raw_pub = rospy.Publisher('raw', String, queue_size=10)

    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)
    rospy.spin()

