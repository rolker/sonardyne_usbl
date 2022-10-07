#!/usr/bin/env python3

''' Filter USBL fixes
'''

import datetime
from project11 import geodesic
import math

class Tracker():
  def __init__(self):
    self.last_position = None
    self.last_timestamp = None

  def add(self, lat, lon, alt, timestamp):
    ret = None
    if alt < 0.0 and alt > -300:
      lat_rad = math.radians(lat)
      lon_rad = math.radians(lon)
      if self.last_position is not None:
        az, distance = geodesic.inverse(self.last_position['lon'], self.last_position['lat'], lon_rad, lat_rad)
        dt = timestamp - self.last_timestamp
        if distance/dt.total_seconds() < 10:
          ret = {'lat':lat, 'lon':lon, 'alt':alt, 'timestamp':timestamp}        
      else:
        ret = {'lat':lat, 'lon':lon, 'alt':alt, 'timestamp':timestamp}        
      self.last_position = {'lat': lat_rad, 'lon': lon_rad}
      self.last_timestamp = timestamp
    return ret





if __name__ == '__main__':
  import sys
  infile = open(sys.argv[1],'r')
  tracker = Tracker()
  for l in infile.readlines():
    #print(l)
    if len(l) and l[0] != '%':
      # decode GeoPointStamped from rostopic echo -p
      parts = l.split(',')
      if len(parts) == 7:
        ts = datetime.datetime.fromtimestamp(float(parts[2])/1000000000.0, datetime.timezone.utc)
        #print (ts,parts)
        p = tracker.add(float(parts[4]), float(parts[5]), float(parts[6]), ts)
        if p is not None:
          print(p['timestamp'].isoformat()+','+str(p['lat'])+','+str(p['lon'])+','+str(p['alt']))
        