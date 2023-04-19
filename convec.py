import h5py
import numpy as np
import hdf5plugin
import roslib;
import rospy
import rosbag
import os
import sys
import argparse
from sensor_msgs.msg import CameraInfo
from dvs_msgs.msg import Event, EventArray

# PATHS TO events.h5 FILES AND outbag
path = "/media/zhouyum/NN/"
filename_l = path + "loop-floor0-events_left.hdf"
filename_r = path + "loop-floor0-events_right.hdf"
outbag = rosbag.Bag( "/media/zhouyum/SH/events.bag", 'w')


n_events = 30000
frequency = 1000
ts_threshold = 1 / frequency

h5f = h5py.File(filename_l, "r")
events = dict()
first_ts = 0

for dset_str in ['p', 'x', 'y', 't']:
  events[dset_str] = h5f['events/{}'.format(dset_str)]

np_x = h5f['events/x']
np_y = h5f['events/y']
np_t = h5f['events/t']
np_p = h5f['events/p']

event_array = EventArray()
cnt = 0
  
for x, y, t, p in zip(np_x, np_y, np_t, np_p):
  ts = float(t) / 1000000
    
  event = Event(x, y, rospy.Time(ts), p)
    
  if len(event_array.events) == 0:
    event_array.events.append(event)
    event_array.header.stamp = rospy.Time(ts)
    event_array.height = 720
    event_array.width = 1280
    first_event_ts = ts
    cnt = 1
  #elif ts - first_ts > ts_threshold:
  elif cnt > n_events:
    outbag.write("/davis/left/events", event_array, event_array.header.stamp)
    event_array.events = [event]
    event_array.header.stamp = rospy.Time(ts)
    event_array.height = 720
    event_array.width = 1280
    cnt = 1
  else:  
    event_array.events.append(event)
    cnt += 1   
      
###############################      


h5f = h5py.File(filename_r, "r")
events = dict()
first_ts = 0

for dset_str in ['p', 'x', 'y', 't']:
  events[dset_str] = h5f['events/{}'.format(dset_str)]

np_x = h5f['events/x']
np_y = h5f['events/y']
np_t = h5f['events/t']
np_p = h5f['events/p']

outbag = rosbag.Bag(outbag,'w')

event_array = EventArray()
cnt = 0
  
for x, y, t, p in zip(np_x, np_y, np_t, np_p):
  ts = float(t) / 1000000
    
  event = Event(x, y, rospy.Time(ts), p)
    
  if len(event_array.events) == 0:
    event_array.events.append(event)
    print("adding events")
    event_array.header.stamp = rospy.Time(ts)
    event_array.height = 480
    event_array.width = 640
    first_event_ts = ts
    cnt = 1
  #elif ts - first_ts > ts_threshold:
  elif cnt > n_events:
    outbag.write("/davis/right/events", event_array, event_array.header.stamp)
    event_array.events = [event]
    event_array.header.stamp = rospy.Time(ts)
    event_array.height = 480
    event_array.width = 640
    cnt = 1
  else:  
    event_array.events.append(event)
    cnt += 1   
         
outbag.close()