#!/usr/bin/env python3
"""Own a single demo and clean all of its descendants, including gz Ruby workers."""
import fcntl
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

workspace = Path(__file__).resolve().parent.parent
domain = str(int(os.environ.get('ROS_DOMAIN_ID', '0')))
lock = (workspace / 'log' / f'demo-{domain}.lock').open('w')
try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit(f'ROS_DOMAIN_ID={domain} 已有一个演示实例运行，请先在启动终端按 Ctrl+C。')
process = subprocess.Popen(['ros2', 'launch', 'workshop_navigation', 'go1_mujoco.launch.py', *sys.argv[1:]])
children = {}

def process_info(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return int(stat[1]), stat[19]
    except (OSError, ValueError, IndexError):
        return None

def snapshot():
    parents = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            info = process_info(entry.name)
            if info:
                parents[int(entry.name)] = info
    pending, seen = [process.pid], set()
    while pending:
        parent = pending.pop()
        if parent in seen:
            continue
        seen.add(parent)
        for pid, (ppid, started) in parents.items():
            if ppid == parent:
                children[pid] = started
                pending.append(pid)

def signal_children(sig):
    for pid, started in children.items():
        info = process_info(pid)
        if info and info[1] == started:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

def stop(signum=None, frame=None):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    snapshot()
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    signal_children(signal.SIGINT)
    time.sleep(2)
    signal_children(signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    time.sleep(.2)
    signal_children(signal.SIGKILL)
    raise SystemExit(0 if signum else (process.returncode or 0))

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while process.poll() is None:
    snapshot()
    time.sleep(.3)
stop()
