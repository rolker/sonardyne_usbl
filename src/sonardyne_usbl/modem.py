#!/usr/bin/env python3

import socket
import serial

class TCPConnection:
  def __init__(self, host, port=4000):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.settimeout(15)
    self.sock.connect((host,port))

  def send(self, data):
    self.sock.send(data.encode('utf-8'))
    return self.sock.recv(2048).decode('utf-8')

class SerialConnection:
  def __init__(self, port, baud=9600):
    self.port = serial.Serial(port, baud)
    self.port.timeout = 15

  def send(self, data):
    self.port.write(data.encode('utf-8'))
    return self.port.readline().decode('utf-8')

class Modem:
  def __init__(self, connection):
    self.connection = connection

  def enableDiagnostics(self):
    return self.connection.send('DIAG:XC,TEL,FEC\n')

  def disableDiagnostics(self):
    return self.connection.send('DIAG:NONE\n')

  def hardwareSelfTest(self):
    return self.connection.send('CKHW\n')

  def getStatus(self, cmd, processCallback, address=None):
    send_cmd = cmd
    if address is not None:
      send_cmd += ':'+str(address)
    data = self.connection.send(send_cmd+'\n')
    diag = None
    if '[' in data:
      data,diag = data.strip().split('[',1)
    
    ret = {}
      
    for part in data.strip().split(','):
      processCallback(part, ret)

    if diag is not None:
      ret['diag'] = diag[:-1]

    return ret

  def processFixedStatus(self, part, ret):
    if part.startswith('>FS:'):
      ret['address'] = part.split(':',1)[1]
    if len(part) == 7 and part[0] == 'U':
      ret['unit_id'] = part[1:]
    if len(part) > 2 and part[:2] == 'FV':
      ret['firmware_version'] = part[2:]
    if len(part) > 4 and part[:4] == 'TDR;':
      ret['transducer_information'] = part[4:]

  def fixedStatus(self, address=None):
    return self.getStatus('FS', self.processFixedStatus, address)


  def processConfigStatus(self, part, ret):
    #print (part)
    if part.startswith('>CS:'):
      ret['address'] = part.split(':',1)[1]
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
    

  def configStatus(self, address=None):
    return self.getStatus('CS', self.processConfigStatus, address)

  def processModemStatus(self, part, ret):
    #print (part)
    if part.startswith('>MS:'):
      ret['address'] = part.split(':',1)[1]
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
    
  def volotileStatus(self, address=None):
    return self.getStatus('VS', self.processVolotileStatus, address)

  def processVolotileStatus(self, part, ret):
    #print (part)
    if part.startswith('>VS:'):
      ret['address'] = part.split(':',1)[1]
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

        

  def sendMessage(self, address, data):
    data = self.connection.send('MDFT:'+address+'|'+data+'\n')
    return data



  def modemStatus(self):
    return self.getStatus('MS', self.processModemStatus)

if __name__ == '__main__':
  import sys
  if sys.argv[1] == 'serial':
    c = SerialConnection(sys.argv[2])
  if sys.argv[1] == 'tcp':
    c = TCPConnection(sys.argv[2])
  m = Modem(c)
  print (m.hardwareSelfTest())
  print (m.enableDiagnostics())
  if len(sys.argv) > 3:
    print('fixed status:', m.fixedStatus(sys.argv[3]))
    print('config status:', m.configStatus(sys.argv[3]))
    print('volotile status:', m.volotileStatus(sys.argv[3]))
    print('modem status:', m.modemStatus())
  else:
    print('fixed status:', m.fixedStatus())
    print('config status:', m.configStatus())
    print('volotile status:', m.volotileStatus())
    print('modem status:', m.modemStatus())
  if len(sys.argv) > 4:
    print(m.sendMessage(sys.argv[3], sys.argv[4]))
