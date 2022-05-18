#!/usr/bin/env python3

from urllib import response
import rospy

from std_msgs.msg import String
from sonardyne_msgs.msg import SMS, SMSResponse

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

import sonardyne_usbl.modem

message_pub = None
sms_response_pub = None
raw_pub = None
modem = None

def timerCallback(event):
    data = modem.getResponse()
    if data is not None:
        print (data)
        raw_pub.publish(String(data['raw']))
        if 'response_type' in data and data['response_type'] == 'SMS':
            if 'message' in data:
                sms = SMS()
                sms.receive_time = rospy.Time.now()
                sms.address = data['address']
                sms.message = data['message']
                message_pub.publish(sms)
            elif 'response' in data:
                sms_reply = SMSResponse()
                sms_reply.receive_time = rospy.Time.now()
                sms_reply.address = data['address']
                sms_reply.response = data['response']
                if 'time_of_flight' in data:
                    sms_reply.time_of_flight = data['time_of_flight']
                sms_response_pub.publish(sms_reply)
                
        if 'diag' in data:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = rospy.Time.now()

            ds = DiagnosticStatus()
            ds.name = rospy.get_name()
            for k in data['diag'].keys():
                ds.values.append(KeyValue(k,data['diag'][k]))
            diag_array.status.append(ds)
            diagnostic_pub.publish(diag_array)



def sendSMSCallback(msg):
    modem.sendSMS(msg.address, msg.message)

def sendRawCallback(msg):
    modem.send(msg.data+'\n')

if __name__ == '__main__':
    rospy.init_node('sonardyne_modem')
  
    connection_type = rospy.get_param('~connection/type')
    if connection_type == 'serial':
        port = rospy.get_param('~connection/port')
        baud_rate = rospy.get_param('~connection/baud_rate', 9600)
        c = sonardyne_usbl.modem.SerialConnection(port, baud_rate)
    elif connection_type == 'tcp':
        host = rospy.get_param('~connection/host')
        port = rospy.get_param('~connection/port', 4000)
        c = sonardyne_usbl.modem.TCPConnection(host, port)
    elif connection_type == 'udp':
        host = rospy.get_param('~connection/host')
        input_port = rospy.get_param('~connection/input_port', 50011)
        output_port = rospy.get_param('~connection/outport_port', 50010)
        c = sonardyne_usbl.modem.UDPConnection(input_port, output_port, host)

    modem = sonardyne_usbl.modem.Modem(c)
    modem.enableDiagnostics()

    message_pub = rospy.Publisher('~received_sms', SMS, queue_size=10)
    sms_response_pub = rospy.Publisher('~sms_response', SMSResponse, queue_size=10)
    raw_pub = rospy.Publisher('~raw', String, queue_size=10)

    send_sms_sub = rospy.Subscriber('~send_sms', SMS, sendSMSCallback, queue_size=10)
    send_raw_sub = rospy.Subscriber('~send_raw', String, sendRawCallback, queue_size=10)
    
    diagnostic_pub = rospy.Publisher('/diagnostics',DiagnosticArray,queue_size=10)

    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)
    rospy.spin()

