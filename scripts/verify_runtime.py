#!/usr/bin/env python3
"""Observe actual Gazebo odometry and scan data; write inspectable evidence."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=25.);p.add_argument('--output',default='log/runtime_evidence.json');args=p.parse_args()
rclpy.init();n=Node('workshop_verification');e={'odom_samples':0,'scan_samples':0,'max_displacement_m':0.,'min_height_m':None,'max_height_m':None,'states':[]};first=None;t0=time.monotonic();e['trajectory']=[]

def odom(m):
 global first
 q=m.pose.pose.position
 if first is None:first=(q.x,q.y);e['start']=[q.x,q.y,q.z]
 e['trajectory'].append([round(time.monotonic()-t0,3),round(q.x,3),round(q.y,3),round(q.z,3)]);e['end']=[q.x,q.y,q.z];e['odom_samples']+=1;e['max_displacement_m']=max(e['max_displacement_m'],math.hypot(q.x-first[0],q.y-first[1]))
 e['min_height_m']=min(e['min_height_m'] if e['min_height_m'] is not None else q.z,q.z);e['max_height_m']=max(e['max_height_m'] or q.z,q.z)
def scan(m):
 e['scan_samples']+=1;e['scan_frame']=m.header.frame_id;e['scan_rays']=len(m.ranges)
def status(m):
 s=json.loads(m.data)['state']
 if s not in e['states']:e['states'].append(s)
n.create_subscription(Odometry,'/odom',odom,10);n.create_subscription(LaserScan,'/scan',scan,10);n.create_subscription(String,'/workshop/status',status,10)
start=time.monotonic()
while time.monotonic()-start<args.duration:rclpy.spin_once(n,timeout_sec=.1)
e['observed_seconds']=time.monotonic()-start;e['motion_verified']=e['max_displacement_m']>.25 and e['scan_samples']>0
Path(args.output).write_text(json.dumps(e,indent=2));print(json.dumps({k:v for k,v in e.items() if k!='trajectory'},indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if e['motion_verified'] else 1)
