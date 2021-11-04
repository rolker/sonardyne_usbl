#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from geographic_msgs.msg import GeoPointStamped
from sonardyne_msgs.msg import SMS
from sonardyne_msgs.msg import Position
from sonardyne_msgs.msg import DeviceStatus
from sonardyne_msgs.msg import DeviceEnable

import xml.etree.ElementTree as ET
import datetime

import sonardyne_usbl.ranger2

position_pubs = {}

def processRemoteControl(data):
    now = rospy.Time.now()
    root = ET.fromstring(data)
    for element in root:
        if element.tag == 'AsynchStatus':
            for sube in element:
                if sube.tag == 'GeographicPositions':
                    for gp in sube:
                        geoPoint = GeoPointStamped()
                        geoPoint.position.altitude = -float(gp.attrib['Depth'])
                        geoPoint.position.latitude = float(gp.attrib['Latitude'])
                        geoPoint.position.longitude = float(gp.attrib['Longitude'])
                        name = gp.attrib['Name']
                        validTime = gp.attrib['TimeOfValidity']
                        timeParts = validTime.split(':')
                        secs = float(timeParts[2])
                        secs_int = int(secs)
                        usec = int((secs-secs_int)*1000000)
                        ts = rospy.Time.now()
                        dt = datetime.datetime.utcfromtimestamp(ts.to_sec())
                        dt = datetime.datetime(dt.year, dt.month, dt.day, int(timeParts[0]), int(timeParts[1]), secs_int, usec)

                        geoPoint.header.stamp = rospy.Time.from_sec(dt.timestamp())
                        
                        if not name in position_pubs:
                            position_pubs[name] = rospy.Publisher("~positions/"+name, GeoPointStamped, queue_size=1)
                        position_pubs[name].publish(geoPoint)
    for gp in root.iter('GeographicPositions'):
        for p in gp.iter('Position'):
            position_msg = Position()
            position_msg.header.stamp = now
            position_msg.UID = p.attrib['UID']
            position_msg.age = float(p.attrib['Age'])
            position_msg.category = p.attrib['Category']
            position_msg.name = p.attrib['Name']
            position_msg.latitude = float(p.attrib['Latitude'])
            position_msg.longitude = float(p.attrib['Longitude'])
            position_msg.depth = float(p.attrib['Depth'])
            position_msg.history = p.attrib['History']
            geographic_positions_pub.publish(position_msg)
    for ds in root.iter('DeviceStatus'):
        for d in ds.iter('Device'):
            try:
                status = DeviceStatus()
                status.header.stamp = now
                status.UID = d.attrib['UID']
                status.name = d.attrib['Name']
                status.type = d.attrib['Type']
                status.state = int(d.attrib['State'])
                device_status_pub.publish(status)
            except KeyError:
                pass
    for ds in root.iter('DeviceStatus'):
        device = None
        for item in ds:
            if item.tag == 'Device':
                if 'ActionType' in  item.attrib and item.attrib['ActionType'] == 'LINK STATUS' and 'Name' in item.attrib:
                    device = item.attrib['Name']
                else:
                    device = None
            if device is not None and item.tag == 'Properties':
                print (device, item.attrib)



def processPAN(data):
    parts = data.strip().split('|',1)
    data = parts[0]

    if '[' in data:
        data,diag = data.strip().split('[',1)

    if len(parts) == 2:
        msg = parts[1]

        data_parts = data.split(',')
        for p in data_parts:
            if p.startswith('SMS:'):
                address = p.split(':')[1]
                sms = SMS()
                sms.address = address
                sms.message = msg
                received_sms_pub.publish(sms)



def timerCallback(event):
    data = rc.getData()
    if data is not None:
        #print(data)
        raw_control_pub.publish(String(data.decode('utf8')))
        processRemoteControl(data)
    data = pan.getData()
    if data is not None:
        #print('from pan:', data)
        raw_pan_pub.publish(String(data.decode('utf8')))
        processPAN(data.decode('utf8'))


def statusCallback(event):
    rc.getAllLinkStatus()

def sendSMSCallback(msg):
    msg_str = "<SMS:"+msg.address+"|"+msg.message+"\n"
    pan.send(msg_str.encode('utf8'))

def sendRawPANCallback(msg):
    msg_str = msg.data+'\n'
    print ('sending:', msg_str)
    pan.send(msg_str.encode('utf8'))


def enableRemoteCallback(msg):
    rc.enableRemote(msg.data)

def enableTrackingCallback(msg):
    rc.enableTracking(msg.UID, msg.enable)


if __name__ == '__main__':
    rospy.init_node('sonardyne_ranger')
  
    host = rospy.get_param('~host')
    control_port_in = rospy.get_param('~control_port_in', 50001)
    control_port_out = rospy.get_param('~control_port_out', 50000)
    pan_port_in = rospy.get_param('~pan_port_in', 50011)
    pan_port_out = rospy.get_param('~pan_port_out', 50010)

    rc = sonardyne_usbl.ranger2.RemoteConnection(control_port_in, control_port_out, host)
    pan = sonardyne_usbl.ranger2.ProgrammableAcousticNavigation(pan_port_in, pan_port_out, host)

    rc.getStatus()
    rc.enableRemote()
    rc.enableAsync(True)

    raw_control_pub = rospy.Publisher('~raw_control', String, queue_size=10)
    raw_pan_pub = rospy.Publisher('~raw_pan', String, queue_size=10)

    received_sms_pub = rospy.Publisher('~received_sms', SMS, queue_size=10)
    geographic_positions_pub = rospy.Publisher('~geographic_positions', Position, queue_size=50)
    device_status_pub = rospy.Publisher('~device_status', DeviceStatus, queue_size=50)

    send_sms_sub = rospy.Subscriber('~send_sms', SMS, sendSMSCallback, queue_size=10)
    enable_remote_sub = rospy.Subscriber('~enable_remote', Bool, enableRemoteCallback, queue_size=1)
    enable_tracking_sub = rospy.Subscriber('~enable_tracking', DeviceEnable, enableTrackingCallback, queue_size=1)
    send_raw_pan_sub = rospy.Subscriber('~send_raw_pan', String, sendRawPANCallback, queue_size=10)


    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)
    #status_timer = rospy.Timer(rospy.Duration(5), statusCallback)

    rospy.spin()
