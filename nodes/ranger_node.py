#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

import xml.etree.ElementTree as ET
import datetime

import sonardyne_usbl.ranger2


position_pubs = {}

def processRemoteControl(data):
    root = ET.fromstring(data)
    for element in root:
        print (element)
        if element.tag == 'AsynchStatus':
            for sube in element:
                if sube.tag == 'GeographicPositions':
                    for gp in sube:
                        print ('gp', gp)
                        print (gp.attrib)
                        nsf = NavSatFix()
                        nsf.altitude = -float(gp.attrib['Depth'])
                        nsf.latitude = float(gp.attrib['Latitude'])
                        nsf.longitude = float(gp.attrib['Longitude'])
                        name = gp.attrib['Name']
                        validTime = gp.attrib['TimeOfValidity']
                        timeParts = validTime.split(':')
                        secs = float(timeParts[2])
                        secs_int = int(secs)
                        usec = int((secs-secs_int)*1000000)
                        ts = rospy.Time.now()
                        dt = datetime.datetime.utcfromtimestamp(ts.to_sec())
                        dt = datetime.datetime(dt.year, dt.month, dt.day, int(timeParts[0]), int(timeParts[1]), secs_int, usec)

                        nsf.header.stamp = rospy.Time.from_sec(dt.timestamp())
                        
                        if not name in position_pubs:
                            position_pubs[name] = rospy.Publisher("~positions/"+name, NavSatFix, queue_size=1)
                        position_pubs[name].publish(nsf)
 
def timerCallback(event):
    data = rc.getData()
    if data is not None:
        #print(data)
        raw_control_pub.publish(String(data.decode('utf8')))
        processRemoteControl(data)
    data = pan.getData()
    if data is not None:
        #print('from pan:', data)
        raw_pan_pub.publish(String(data))

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

    raw_control_pub = rospy.Publisher('raw_control', String, queue_size=10)
    raw_pan_pub = rospy.Publisher('raw_pan', String, queue_size=10)


    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)
    rospy.spin()
