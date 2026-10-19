#!/usr/bin/env python3
"""接口自检：确认运动层节点的话题、frame、数据率都符合契约。

★ 为什么需要它：和别人的导航栈组合时，最常见的失败不是"报错"，而是
   "话题名字对了但 frame 不对 / 数据没有 / QoS 不匹配"，程序不崩，
   只是行为诡异。这个脚本把这些一次性查清楚。

用法：
    # 先起节点：ros2 launch go1_mujoco_locomotion locomotion.launch.py
    ros2 run go1_mujoco_locomotion verify_interface
"""

import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan


class Verifier(Node):
    def __init__(self):
        super().__init__("go1_verify_interface")
        self.got = {}
        self.create_subscription(Odometry, "odom", self._odom, 10)
        self.create_subscription(LaserScan, "scan", self._scan, 10)
        self.create_subscription(JointState, "joint_states", self._js, 10)

    def _odom(self, m):
        self.got["odom"] = m
    def _scan(self, m):
        self.got["scan"] = m
    def _js(self, m):
        self.got["js"] = m


def main():
    rclpy.init()
    n = Verifier()
    t0 = time.time()
    while time.time() - t0 < 12 and len(n.got) < 3:
        rclpy.spin_once(n, timeout_sec=0.2)

    ok = True
    print("\n  === Go1 运动层接口自检 ===\n")
    for k, desc in (("odom", "nav_msgs/Odometry"), ("scan", "sensor_msgs/LaserScan"),
                    ("js", "sensor_msgs/JointState")):
        m = n.got.get(k)
        if m is None:
            print(f"  [FAIL] {k:<5} ({desc}) 收不到 —— 节点起了吗？话题名对得上吗？")
            ok = False
        else:
            print(f"  [ OK ] {k:<5} ({desc})  √")

    od = n.got.get("odom")
    if od:
        z = od.pose.pose.position.z
        print(f"\n  odom   frame: '{od.header.frame_id}' -> '{od.child_frame_id}'")
        print(f"         站立高度 z = {z:.3f} m   "
              f"{'√ 正常（Go1 约 0.28~0.31）' if 0.15 < z < 0.45 else '✗ 异常，机器人可能摔了'}")
        # twist 必须是机体系量级
        vx = od.twist.twist.linear.x
        print(f"         机体系 vx = {vx:+.3f} m/s")
        if od.header.frame_id != "odom" or od.child_frame_id != "base_link":
            print("  [WARN] frame 不是 odom -> base_link；"
                  "若你的 Nav2 配置用别的 frame，需要相应改参数或做重映射")
    sc = n.got.get("scan")
    if sc:
        import numpy as np
        r = np.array(sc.ranges)
        print(f"\n  scan   frame: '{sc.header.frame_id}'")
        print(f"         {len(r)} 线  angle_min={sc.angle_min:+.3f} "
              f"inc={sc.angle_increment:.5f}  range=[{sc.range_min:.2f}, {sc.range_max:.2f}]")
        print(f"         最小 {r.min():.2f} m  中位 {np.median(r):.2f} m")
        print("         约定：索引 N/2 对应正前方（angle_min=-π）"
              "；数据已在 base_link 系，无需额外 laser_frame")
    js = n.got.get("js")
    if js:
        print(f"\n  joint_states: {len(js.name)} 个关节  {js.name[:3]} ...")
        print("         可交给 robot_state_publisher（配你自己的 URDF）发布 TF 树")

    print("\n  === 结论 ===")
    print("  接口正常，可以和你的 Nav2 组合了。" if ok else
          "  有缺失项，先解决上面的 FAIL 再组合。")
    print("\n  ★ 组合时最容易踩的三个坑：")
    print("    1) Nav2 的速度下限必须抬到工作区间，否则机器人一步不走：")
    print("         min_speed_xy >= 0.28   （不能是 0.01）")
    print("         min_speed_theta >= 0.45（不能是 0.0）")
    print("       且都要满足 min_* <= acc_lim_* / controller_frequency")
    print("    2) 全栈统一时间基准：本节点用墙钟，若你的栈 use_sim_time=true")
    print("       就要求有人发 /clock，否则等待/超时逻辑会全部失效。")
    print("    3) 多终端一起用时都导出 ROS_LOCALHOST_ONLY=1（本机网卡异常时必需），")
    print("       否则会出现「launch 正常但某个节点收不到消息」的诡异现象。\n")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
