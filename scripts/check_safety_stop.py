#!/usr/bin/env python3
"""Verify zero safe command and no obstacle contact in the blocked aisle demo."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init();n=Node('stop_verifier');poses=[];speeds=[]
n.create_subscription(Odometry,'/odom',lambda m:poses.append([m.pose.pose.position.x,m.pose.pose.position.y]),10)
n.create_subscription(Twist,'/cmd_vel_safe',lambda m:speeds.append(abs(m.linear.x)+abs(m.linear.y)+abs(m.angular.z)),10)
t0=time.monotonic()
while time.monotonic()-t0<6:rclpy.spin_once(n,timeout_sec=.1)
e={'samples':len(poses),'max_safe_speed':max(speeds,default=999),'displacement_m':math.dist(poses[0],poses[-1]) if poses else 999,'final_position':poses[-1] if poses else None}
e['stopped_before_obstacle']=bool(poses) and poses[-1][0]<-5.82 and e['max_safe_speed']<.01 and e['displacement_m']<.05
Path('log/blocked_stop_evidence.json').write_text(json.dumps(e,indent=2));print(json.dumps(e,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if e['stopped_before_obstacle'] else 1)
