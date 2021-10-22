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

    def send(self, msg):
        crc = binascii.crc32(msg)
        crc_packed = struct.pack('>I', crc)
        crc_unpacked, = struct.unpack('<I', crc_packed)
        crc_hex = hex(crc_unpacked)[2:]
        while len(crc_hex) < 8:
            crc_hex = '0'+crc_hex
        payload = b'\x10\x02'+msg+ crc_hex.upper().encode('utf8')+b'\x10\x03'
        self.out_socket.sendto(payload, self.remote_address)

    def spinOnce(self):
        try:
            data = self.in_socket.recv(4096)
            if len(data) > 12:
                payload = data[2:-10]
                root = ET.fromstring(payload)
                printXMLNode(root)
        except socket.timeout:
            pass

if __name__ == '__main__':
    rc = RemoteConnection(50001, 50000, 'survey_pc')
    rc.send(b'<RemoteControl ProtocolVersion="1.5"><Get><Job /></Get></RemoteControl>')
    while True:
        rc.spinOnce()