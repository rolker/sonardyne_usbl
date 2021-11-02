#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from sonardyne_msgs.msg import SMS

import sonardyne_usbl.modem

message_pub = None
raw_pub = None
modem = None

def timerCallback(event):
    data = modem.getResponse()
    if data is not None:
        print (data)
        raw_pub.publish(String(data['raw']))
        if data['response_type'] == 'SMS' and 'message' in data:
            sms = SMS()
            sms.address = data['address']
            sms.message = data['message']
            message_pub.publish(sms)

def sendSMSCallback(msg):
    modem.sendSMS(msg.address, msg.message)

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

    message_pub = rospy.Publisher('messages', SMS, queue_size=10)
    raw_pub = rospy.Publisher('raw', String, queue_size=10)

    send_sms_sub = rospy.Subscriber('~send_sms', SMS, sendSMSCallback, queue_size=10)

    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)
    rospy.spin()

