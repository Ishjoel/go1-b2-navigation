#!/usr/bin/env python3
"""Insert a real collision object into the running Harmonic world."""
import argparse
import subprocess
import tempfile
import re
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--x',type=float,default=-3.);p.add_argument('--y',type=float,default=0.);p.add_argument('--width',type=float,default=.8);p.add_argument('--name',default='temporary_obstacle');p.add_argument('--remove',action='store_true');a=p.parse_args()
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*',a.name):p.error('invalid model name')
    if a.remove:
        return subprocess.call(['gz','service','-s','/world/workshop/remove','--reqtype','gz.msgs.Entity','--reptype','gz.msgs.Boolean','--timeout','3000','--req',f'name: "{a.name}" type: MODEL'])
    if a.width<=0: p.error('width must be positive')
    sdf=f'<sdf version="1.9"><model name="{a.name}"><static>true</static><pose>{a.x} {a.y} 0.6 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>0.6 {a.width} 1.2</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.6 {a.width} 1.2</size></box></geometry><material><diffuse>1 0.25 0.05 1</diffuse></material></visual></link></model></sdf>'
    with tempfile.TemporaryDirectory() as tmp:
        f=Path(tmp)/'obstacle.sdf';f.write_text(sdf)
        return subprocess.call(['ros2','run','ros_gz_sim','create','-world','workshop','-file',str(f),'-name',a.name,'-x',str(a.x),'-y',str(a.y),'-z','0.6'])
if __name__=='__main__':raise SystemExit(main())
