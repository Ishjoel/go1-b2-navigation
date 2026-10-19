#!/usr/bin/env python3
"""Measured odometry TF, command watchdog, operator stop and RViz status."""
import copy
import time
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

class Runtime(Node):
    def __init__(self):
        super().__init__('workshop_runtime')
        self.tf = TransformBroadcaster(self)
        self.last_cmd = 0.; self.last_odom = 0.; self.command = Twist(); self.stopped = False
        self.pose = None
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.status = self.create_publisher(String, '/workshop/status', 10)
        self.marker = self.create_publisher(Marker, '/workshop/status_marker', 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd, 10)
        self.create_service(SetBool, '/workshop/emergency_stop', self.stop)
        self.create_timer(.05,self.tick)
    def odom(self,msg):
        self.last_odom=time.monotonic(); self.pose=msg.pose.pose
        t=TransformStamped(); t.header=msg.header; t.header.frame_id='odom'; t.child_frame_id='base_link'
        t.transform.translation.x=self.pose.position.x; t.transform.translation.y=self.pose.position.y; t.transform.translation.z=self.pose.position.z
        t.transform.rotation=self.pose.orientation; self.tf.sendTransform(t)
    def cmd(self,msg):
        self.command=msg; self.last_cmd=time.monotonic()
    def stop(self,req,res):
        self.stopped=req.data; res.success=True; res.message='Emergency stop' if req.data else 'Released'; return res
    def tick(self):
        now=time.monotonic()
        active=not self.stopped and now-self.last_cmd<.5 and now-self.last_odom<.5
        cmd=self.command if active else Twist()
        self.pub.publish(cmd)
        state='EMERGENCY_STOP' if self.stopped else ('MOVING' if active and abs(cmd.linear.x)+abs(cmd.angular.z)>.01 else 'STOPPED')
        self.status.publish(String(data=json.dumps({'state':state,'odom_fresh':now-self.last_odom<.5,'speed':cmd.linear.x})))
        if self.pose:
            m=Marker();m.header.frame_id='odom';m.header.stamp=self.get_clock().now().to_msg();m.ns='status';m.id=0;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose=copy.deepcopy(self.pose);m.pose.position.z+=1.;m.scale.z=.25;m.color.a=1.;m.color.r=.2;m.color.g=1.;m.color.b=.5;m.text='Unitree Go1 / MuJoCo / Jazzy\n'+state;self.marker.publish(m)

def main():
    rclpy.init();node=Runtime()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.pub.publish(Twist())
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__': main()
