#!/usr/bin/env python3

import socket

class Modem:
  def __init__(self, host, port=4000):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.settimeout(1)
    self.sock.connect((host,port))

  def fixedParameters(self):
    self.sock.send('FS\n'.encode('utf-8'))
    parts = self.sock.recv(1024).decode('utf-8').split(',')
    ret = {}
    for p in parts:
      if len(p) == 7 and p[0] == 'U':
        ret['UID'] = p
      if len(p) > 2 and p[:2] == 'FV':
        ret['FV'] = p
      if len(p) > 2 and p[:3] == 'TDR':
        ret['TDR'] = p

    return ret

if __name__ == '__main__':
  import sys
  m = Modem(sys.argv[1])
  print(m.fixedParameters())
