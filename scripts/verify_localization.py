#!/usr/bin/env python3
import json
import math
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from pathlib import Path
rclpy.init();n=Node('localization_verifier');poses={}
n.create_subscription(Odometry,'/odom',lambda m:poses.update(odom=m.pose.pose),10)
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:poses.update(amcl=m.pose.pose),10)
t0=time.monotonic()
while len(poses)<2 and time.monotonic()-t0<15:rclpy.spin_once(n,timeout_sec=.2)
e={}
for k,v in poses.items():e[k]=[v.position.x,v.position.y]
if len(poses)==2:e['position_error_m']=math.dist(e['odom'],e['amcl'])
Path('log/localization_evidence.json').write_text(json.dumps(e,indent=2));print(json.dumps(e,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if e.get('position_error_m',999)<.5 else 1)
