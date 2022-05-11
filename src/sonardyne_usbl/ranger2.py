#!/usr/bin/env python3

import socket
import binascii
import struct
import xml.etree.ElementTree as ET

def printXMLNode(node, tab=''):
    print (tab, node.tag, node.attrib)
    for child in node:
        printXMLNode(child, tab+'  ')

class RemoteConnection:
    def __init__(self, inport, outport, remote_host, local_address=''):
        
        self.in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.in_socket.bind((local_address, inport))
        self.in_socket.settimeout(0.1)

        self.out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_address = (remote_host, outport)

        self.beacon_uids = {}

    def send(self, msg):
        crc = binascii.crc32(msg)
        crc_packed = struct.pack('>I', crc)
        crc_unpacked, = struct.unpack('<I', crc_packed)
        crc_hex = hex(crc_unpacked)[2:]
        while len(crc_hex) < 8:
            crc_hex = '0'+crc_hex
        payload = b'\x10\x02'+msg+ crc_hex.upper().encode('utf8')+b'\x10\x03'
        self.out_socket.sendto(payload, self.remote_address)

    def getStatus(self):
        self.send(b'<RemoteControl ProtocolVersion="1.5"><Get><Job/></Get></RemoteControl>')

    def getLinkStatus(self, name):
        if name in self.beacon_uids:
            cmd = '<RemoteControl ProtocolVersion="1.5"><DeviceAction><Job><Objects><Object Type="Beacon" UID="'+self.beacon_uids[name]+'"><Action ActionType="LINK STATUS" /></Object></Objects></Job></DeviceAction></RemoteControl>'
            self.send(cmd.encode('utf8'))

    def getAllLinkStatus(self):
        for name in self.beacon_uids:
            self.getLinkStatus(name)

    def enableTransceiver(self, uid, enable=True):
        if enable:
            cmd = '<RemoteControl ProtocolVersion="1.5"><Set><Job><Objects><Object Type="Transceiver" UID="'+uid+'"><Properties TxRxMode="TxRx"/></Object></Objects></Job></Set></RemoteControl>'
        else:
            cmd = '<RemoteControl ProtocolVersion="1.5"><Set><Job><Objects><Object Type="Transceiver" UID="'+uid+'"><Properties TxRxMode="Disabled"/></Object></Objects></Job></Set></RemoteControl>'
        self.send(cmd.encode('utf8'))

    def enableRemote(self, enable=True):
        if enable:
            self.send(b'<RemoteControl ProtocolVersion="1.5"><Set><Job RemoteControlEnabled="true"/></Set></RemoteControl>')
        else:
            self.send(b'<RemoteControl ProtocolVersion="1.5"><Set><Job RemoteControlEnabled="false"/></Set></RemoteControl>')

    def enableTracking(self, uid, enable=True):
        if enable:
            cmd = '<RemoteControl ProtocolVersion="1.5"><Set><Job><Objects><Object Type="Beacon" UID="'+uid+'"><Properties State="Tracked"/></Object></Objects></Job></Set></RemoteControl>'
        else:
            cmd = '<RemoteControl ProtocolVersion="1.5"><Set><Job><Objects><Object Type="Beacon" UID="'+uid+'"><Properties State="NotTracked"/></Object></Objects></Job></Set></RemoteControl>'
        self.send(cmd.encode('utf8'))


    def enableAsync(self, enable=True):
        if enable:
            self.send(b'<RemoteControl ProtocolVersion="1.5"><Set><Job AsyncGeographicPositionOutputEnabled="True" AsyncGridPositionOutputEnabled="False" AsyncShipRelativePositionOutputEnabled="False" AsyncDeviceStatusOutputEnabled="True" AsyncAlarmOutputEnabled="True" AsyncEventOutputEnabled="False" AsyncOutputInterval="2"></Job></Set></RemoteControl>')
        else:
            self.send(b'<RemoteControl ProtocolVersion="1.5"><Set><Job AsyncGeographicPositionOutputEnabled="False" AsyncGridPositionOutputEnabled="False" AsyncShipRelativePositionOutputEnabled="False" AsyncDeviceStatusOutputEnabled="False" AsyncAlarmOutputEnabled="False" AsyncEventOutputEnabled="False" AsyncOutputInterval="2"></Job></Set></RemoteControl>')

    def setTransceiverGain(self, uid, value):
        cmd = '<RemoteControl ProtocolVersion="1.5"><Set><Job><Objects><Object Type="Beacon" UID="'+uid+'"><Properties TransceiverGain="'+value+'" /></Object></Objects></Job></Set></RemoteControl>'
        self.send(cmd.encode('utf8'))
        
    def getData(self):
        try:
            data = self.in_socket.recv(8192)
            if len(data) > 12:
                payload = data[2:-10]
                #print ('payload size:', len(payload))
                #print('payload:', payload)
                root = ET.fromstring(payload)
                for device in root.iter('Device'):
                    #print (device, device.attrib)
                    if 'Type' in device.attrib and device.attrib['Type'] == 'Beacon':
                        if 'UID' in device.attrib and 'Name' in device.attrib:
                            self.beacon_uids[device.attrib['Name']] = device.attrib['UID']

                return payload
                #root = ET.fromstring(payload)
                #return root
        except socket.timeout:
            pass
        return None

class ProgrammableAcousticNavigation:
    def __init__(self, inport, outport, remote_host, local_address=''):
        
        self.in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.in_socket.bind((local_address, inport))
        self.in_socket.settimeout(0.1)

        self.out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_address = (remote_host, outport)

    def send(self, msg):
        self.out_socket.sendto(msg, self.remote_address)

    def getData(self):
        try:
            return self.in_socket.recv(4096)
        except socket.timeout:
            pass
        return None


if __name__ == '__main__':
    rc = RemoteConnection(50001, 50000, 'survey_pc')
    #rc.send(b'<RemoteControl ProtocolVersion="1.0"><Set><Job><Objects><Object Type="PitchRoll" UID="sjgWjF8QqhA="><Properties Port="COM21"/></Object></Objects></Job></Set></RemoteControl>')
    rc.getStatus()
    rc.enableRemote()
    #rc.spinOnce()
    rc.enableAsync(True)
    # while True:
    #     rc.spinOnce()
    pan = ProgrammableAcousticNavigation(50011, 50010, 'survey_pc')
    pan.send(b"SMS:2001;B0|Hello world!\n")
    while True:
        data = rc.getData()
        if data is not None:
            root = ET.fromstring(data)
            printXMLNode(root)
        data = pan.getData()
        if data is not None:
            print('from pan:', data)
