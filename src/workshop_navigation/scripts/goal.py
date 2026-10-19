#!/usr/bin/env python3
import argparse
import math
import time
import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String

def main():
    parser=argparse.ArgumentParser();parser.add_argument('x',type=float);parser.add_argument('y',type=float);parser.add_argument('--yaw',type=float,default=0.);parser.add_argument('--timeout',type=float,default=180.)
    args=parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:]);rclpy.init();nav=BasicNavigator();pub=nav.create_publisher(String,'/workshop/mission',10)
    # AMCL initial pose is owned by bringup. Avoid BasicNavigator's default
    # initial-pose publication, which would reset a localized robot to (0, 0).
    nav.waitUntilNav2Active(localizer='amcl')
    goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=nav.get_clock().now().to_msg();goal.pose.position.x=args.x;goal.pose.position.y=args.y;goal.pose.orientation.z=math.sin(args.yaw/2);goal.pose.orientation.w=math.cos(args.yaw/2)
    nav.goToPose(goal);start=time.monotonic();last=0.
    while not nav.isTaskComplete():
        if time.monotonic()-start>args.timeout:
            nav.cancelTask()
            while not nav.isTaskComplete():time.sleep(.05)
            break
        if time.monotonic()-last>1.:
            feedback=nav.getFeedback();text=f'NAVIGATING distance={feedback.distance_remaining:.2f}m' if feedback and feedback.distance_remaining>0 else 'NAVIGATING';pub.publish(String(data=text));print(text,flush=True);last=time.monotonic()
        time.sleep(.1)
    result=nav.getResult();text='SUCCEEDED' if result==TaskResult.SUCCEEDED else ('CANCELED' if result==TaskResult.CANCELED else 'FAILED');pub.publish(String(data=text));print(text,flush=True);nav.destroy_node();rclpy.shutdown();return 0 if text=='SUCCEEDED' else 1
if __name__=='__main__':raise SystemExit(main())
