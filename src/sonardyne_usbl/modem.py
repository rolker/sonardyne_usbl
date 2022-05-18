#!/usr/bin/env python3

import socket
import serial
import time

class TCPConnection:
    def __init__(self, host, port=4000):
        self.host = host
        self.port = port
        self.open()


    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.1)
        self.sock.connect((self.host, self.port))

    def send(self, data):
        self.sock.send(data.encode('utf-8'))

    def recv(self):
        return self.sock.recv(10000).decode('utf-8')

class UDPConnection:
    def __init__(self, inport, outport, remote_host, local_address=''):
        self.inport = inport
        self.local_address = local_address
        self.remote_address = (remote_host, outport)
        self.open()


    def open(self):
        self.in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.in_socket.bind((self.local_address, self.inport))
        self.in_socket.settimeout(0.1)

        self.out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg):
        self.out_socket.sendto(msg.encode('utf-8'), self.remote_address)

    def recv(self):
        try:
            return self.in_socket.recv(10000).decode('utf-8')
        except socket.timeout:
            pass
        return None

class SerialConnection:
    def __init__(self, device, baud=9600):
        self.device = device
        self.baud = baud
        self.open()

    def open(self):
        self.port = serial.Serial(self.device, self.baud)
        self.port.timeout = 0.1

    def send(self, data):
        self.port.write(data.encode('utf-8'))
        #return self.recv()

    def recv(self):
        data = self.port.readline().decode('utf-8')
        if len(data):
            return data
        return None

class Modem:
    def __init__(self, connection):
        self.connection = connection

    def send(self, data):
        print('sending:',data)
        self.connection.send(data)

    def recv(self):
        return self.connection.recv()

    def enableDiagnostics(self):
        #self.send('DIAG:XC,DBV,SNR,FEC,TEL,DOP,RX1,IFL\n')
        self.send('DIAG:XC,DBV,SNR,IFL\n')

    def disableDiagnostics(self):
        self.send('DIAG:NONE\n')

    def hardwareSelfTest(self):
        self.send('CKHW\n')

    def getStatus(self, cmd, address=None):
        send_cmd = cmd
        if address is not None:
            send_cmd += ':'+str(address)
        self.send(send_cmd+'\n')

    def getResponse(self):
        raw = self.recv()
        if raw is None:
            return None

        print('raw:',raw)

        ret = {}

        parts = raw.strip().split('|',1)
        data = parts[0]
        if len(parts) == 2:
            msg = parts[1]
            ret['message'] = msg

        diag = None
        if '[' in data:
            data,diag = data.strip().split('[',1)


        response_type = None
        UID = None

        for part in data.strip().split(','):
            if response_type is None:
                if part.startswith('>'):
                    if len(part) == 8 and part[1] == 'U':
                        UID = part[1:]
                    else:
                        response_type = part.split(':',1)[0][1:]
                else:
                    if len(part) == 7 and part[0] == 'U':
                        UID = part
                    else:
                        response_type = part.split(':',1)[0]
                if response_type == 'FS':
                    ret['response_type'] = 'fixed_status'
                elif response_type == 'CS':
                    ret['response_type'] = 'config_status'
                elif response_type == 'MS':
                    ret['response_type'] = 'modem_status'
                elif response_type == 'VS':
                    ret['response_type'] = 'volatile_status'
                else:
                    ret['response_type'] = response_type
                if response_type in ('FS','CS','MS','VS','SMS'):
                    ret['address'] = part.split(':',1)[1]
                if response_type == 'CKHW':
                    ret['test_result'] = part.split(':',1)[1]

            if response_type == 'FS':
                if len(part) == 7 and part[0] == 'U':
                    ret['unit_id'] = part[1:]
                if len(part) > 2 and part[:2] == 'FV':
                    ret['firmware_version'] = part[2:]
                if len(part) > 4 and part[:4] == 'TDR;':
                    ret['transducer_information'] = part[4:]

            if response_type == 'CS':
                if len(part) > 2 and part[:2] == 'LG':
                    ret['gain'] = part[2:]
                if len(part) > 3 and part[:3] == 'NPL':
                    ret['navigation_power_level'] = part[3:]
                if len(part) > 3 and part[:3] == 'SPL':
                    ret['start_power_level'] = part[3:]
                if len(part) > 3 and part[:3] == 'TPL':
                    ret['telemetry_power_level'] = part[3:]
                if len(part) > 3 and part[:3] == 'RXW':
                    ret['receive_wait_time'] = part[3:]

            if response_type == 'MS':
                if len(part) > 2 and part[:2] == 'MV':
                    ret['modem_version'] = part[2:]
                if len(part) > 2 and part[:2] == 'DD':
                    ret['data_delay'] = part[2:]
                if len(part) > 2 and part[:2] == 'MD':
                    ret['modem_delay'] = part[2:]
                if len(part) > 2 and part[:2] == 'UD':
                    ret['uplink_delay'] = part[2:]
                if len(part) > 2 and part[:2] == 'TS':
                    ret['telemetry_scheme'] = part[2:]
                if len(part) > 1 and part[:1] == 'P':
                    ret['port'] = part[1:]
                if len(part) > 2 and part[:2] == 'MR':
                    ret['master_retries'] = part[2:]
                if len(part) > 2 and part[:2] == 'SM':
                    ret['subframes_missed'] = part[2:]
                if len(part) > 3 and part[:3] == 'THR':
                    ret['threshold'] = part[3:]
                if len(part) > 3 and part[:3] == 'ICT':
                    ret['inter_character_time'] = part[3:]
                if len(part) > 2 and part[:2] == 'FQ':
                    ret['forward_queue'] = part[2:]
                if len(part) > 3 and part[:3] == 'MST':
                    ret['master_mode'] = part[3:]
                if len(part) > 2 and part[:2] == 'MU':
                    ret['multi_user'] = part[2:]
                if len(part) > 2 and part[:2] == 'FF':
                    ret['fire_and_forget'] = part[2:]
                if len(part) > 1 and part[:1] == 'B':
                    ret['buffer_size'] = part[1:]
                if len(part) > 1 and part[:1] == 'R':
                    ret['range'] = part[1:]
                if len(part) > 4 and part[:4] == 'DATA':
                    ret['data'] = part[4:]

            if response_type == 'VS':
                if len(part) > 3 and part[:3] == 'WKT':
                    ret['wake_up_tone'] = part[3:]
                if len(part) > 3 and part[:3] == 'HPR':
                    ret['hipap_channel'] = part[3:]
                if part == 'EXT':
                    ret['external_power'] = True
                if part == 'TILT':
                    ret['tilt_over_45'] = True
                if part == 'OV':
                    ret['override_flag'] = True
                if len(part) > 2 and part[:2] == 'BT':
                    battery = {}
                    bparts = part.split(';')
                    if len(bparts) > 1:
                        battery['type'] = bparts[1]
                    for bpart in bparts:
                        if len(bpart) > 2 and bpart[:2] == 'BT':
                            battery['number'] = bpart[2:]
                        if len(bpart) > 3 and bpart[:3] == 'VLT':
                            battery['voltage'] = bpart[3:]
                        if len(bpart) > 3 and bpart[:3] == 'IDC':
                            battery['current'] = bpart[3:]
                        if len(bpart) > 3 and bpart[:3] == 'CAP':
                            cap,remaining = bpart[3:].split('/',1)
                            battery['capacity'] = cap
                            battery['percent_remaining'] = remaining
                        if len(bpart) > 1 and bpart[:1] == 'T':
                            battery['temperature'] = bpart[1:]
                        if bpart == 'DIS':
                            battery['disconnected'] = True
                        if bpart == 'CHG':
                            battery['charging'] = True

                    if not 'batteries' in ret:
                        ret['batteries'] = []
                    ret['batteries'].append(battery)

            if response_type == 'SMS':
                if part[0] == 'R':
                    try:
                        ret['time_of_flight'] = int(part[1:])/1000000.0
                        ret['response'] = 'ack'
                    except ValueError:
                        print("error parsing R:", part)
                if part == 'NO_REPLY':
                    ret['response'] = 'no_reply'
                if part == 'ACK':
                    ret['response'] = 'ack'
                


        if diag is not None:
            ret['diag'] = self.parseDiag(diag[:-1])

        if UID is not None:
            ret['UID'] = UID

        ret['raw'] = raw

        return ret

    def parseDiag(self, diag):
        ret = {}
        parts = diag.split(',')
        for part in parts:
            if part.startswith('XC'):
                ret['cross_correlation'] = part[2:]
            if part.startswith('SNR'):
                ret['signal_to_noise_ratio'] = part[3:]
            if part.startswith('DBV'):
                ret['decibel_volts'] = part[3:]
            
        ret['raw'] = diag
        return ret

    def fixedStatus(self, address=None):
        self.getStatus('FS', address)

    def configStatus(self, address=None):
        self.getStatus('CS', address)

    def volatileStatus(self, address=None):
        self.getStatus('VS', address)

    def sendMessage(self, address, data):
        self.connection.send('MDFT:'+address+'|'+data+'\n')

    def sendSMS(self, address, data):
        self.connection.send('SMS:'+address+';TS2;RTS2,C0,A0,R1|'+data+'\n')

    def modemStatus(self):
        self.getStatus('MS')

if __name__ == '__main__':
    import sys
    if sys.argv[1] == 'serial':
        c = SerialConnection(sys.argv[2])
    if sys.argv[1] == 'tcp':
        c = TCPConnection(sys.argv[2])
    m = Modem(c)
    m.hardwareSelfTest()
    m.enableDiagnostics()
    address = None
    if len(sys.argv) > 3:
        address = sys.argv[3]
    m.fixedStatus(address)
    m.configStatus(address)
    m.volatileStatus(address)
    m.modemStatus()
    if len(sys.argv) > 4:
        m.sendSMS(sys.argv[3], sys.argv[4])

    while True:
        data = m.getResponse()
        if data is not None and len(data):
            print (data)
        else:
            time.sleep(0)