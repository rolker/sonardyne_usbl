#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from geographic_msgs.msg import GeoPointStamped
from sonardyne_msgs.msg import Position
from sonardyne_msgs.msg import DeviceStatus
from sonardyne_msgs.msg import DeviceEnable
from sonardyne_msgs.msg import DeviceValue

import xml.etree.ElementTree as ET
import datetime

import sonardyne_usbl.ranger2

position_pubs = {}

last_sent_position_times = {}

def processRemoteControl(data):
    now = rospy.Time.now()
    root = ET.fromstring(data)
    for element in root:
        if element.tag == 'AsynchStatus':
            for sube in element:
                if sube.tag == 'GeographicPositions':
                    #print (data)
                    for gp in sube:
                        try:
                            geoPoint = GeoPointStamped()
                            geoPoint.position.altitude = -float(gp.attrib['Depth'])
                            geoPoint.position.latitude = float(gp.attrib['Latitude'])
                            geoPoint.position.longitude = float(gp.attrib['Longitude'])
                            uid = gp.attrib['UID']
                            name = gp.attrib['Name']
                            validTime = gp.attrib['TimeOfValidity']
                            history = gp.attrib['History']
                            if not uid in last_sent_position_times or last_sent_position_times[uid] != validTime:
                                timeParts = validTime.split(':')
                                secs = float(timeParts[2])
                                secs_int = int(secs)
                                usec = int((secs-secs_int)*1000000)
                                ts = rospy.Time.now()
                                dt = datetime.datetime.utcfromtimestamp(ts.to_sec())
                                dt = datetime.datetime(dt.year, dt.month, dt.day, int(timeParts[0]), int(timeParts[1]), secs_int, usec)

                                geoPoint.header.stamp = rospy.Time.from_sec(dt.timestamp())
                                
                                name = name.replace(' ','_')

                                if not name in position_pubs:
                                    #print('creating position publisher for', name)
                                    position_pubs[name] = rospy.Publisher("~positions/"+name, GeoPointStamped, queue_size=1)
                                if len(history) and history[-1] == '1':
                                    position_pubs[name].publish(geoPoint)
                                    last_sent_position_times[uid] = validTime
                                # else:
                                #     print ('not sending pos to', name, " with history", history)
                        except KeyError:
                            pass

    for gp in root.iter('GeographicPositions'):
        for p in gp.iter('Position'):
            try:
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
            except KeyError:
                pass
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

def timerCallback(event):
    data = rc.getData()
    if data is not None:
        #print(data)
        raw_control_pub.publish(String(data.decode('utf8')))
        processRemoteControl(data)

def statusCallback(event):
    rc.getAllLinkStatus()

def enableRemoteCallback(msg):
    rc.enableRemote(msg.data)

def enableTransceiverCallback(msg):
    rc.enableRemote(True)
    rc.enableTransceiver(msg.UID, msg.enable)
    rc.enableRemote(False)

def enableTrackingCallback(msg):
    rc.enableRemote(True)
    rc.enableTracking(msg.UID, msg.enable)
    rc.enableRemote(False)

def setTransceiverGainCallback(msg):
    rc.enableRemote(True)
    rc.setTransceiverGain(msg.UID, msg.value)
    rc.enableRemote(False)

if __name__ == '__main__':
    rospy.init_node('sonardyne_ranger')
  
    host = rospy.get_param('~host')
    control_port_in = rospy.get_param('~control_port_in', 50001)
    control_port_out = rospy.get_param('~control_port_out', 50000)
 
    rc = sonardyne_usbl.ranger2.RemoteConnection(control_port_in, control_port_out, host)

    rc.getStatus()
    rc.enableRemote()
    rc.enableAsync(True)
    rc.enableRemote(False)

    raw_control_pub = rospy.Publisher('~raw_control', String, queue_size=10)

    geographic_positions_pub = rospy.Publisher('~geographic_positions', Position, queue_size=50)
    device_status_pub = rospy.Publisher('~device_status', DeviceStatus, queue_size=50)

    enable_remote_sub = rospy.Subscriber('~enable_remote', Bool, enableRemoteCallback, queue_size=1)
    enable_tracking_sub = rospy.Subscriber('~enable_tracking', DeviceEnable, enableTrackingCallback, queue_size=1)
    enable_tranceiver_sub = rospy.Subscriber('~enable_tranceiver', DeviceEnable, enableTransceiverCallback, queue_size=1)
    enable_set_gain_sub = rospy.Subscriber('~set_transceiver_gain', DeviceValue, setTransceiverGainCallback, queue_size=1)

    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)

    rospy.spin()
