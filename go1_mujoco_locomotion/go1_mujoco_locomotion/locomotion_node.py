#!/usr/bin/env python3
"""Go1 四足运动控制节点（MuJoCo 仿真 + 强化学习策略）。

================================================================================
这是一个**独立可复用**的运动层节点：只做「速度指令 -> 四足行走」这一件事，
对外只暴露标准 ROS 2 话题，因此可以直接和任何自己写的 Nav2 / 导航栈组合。

接口契约（唯一需要知道的）：

    订阅  ~/cmd_vel        geometry_msgs/Twist       期望的机体速度
                                                      linear.x  前进 m/s   （支持 0.3 ~ 1.0）
                                                      linear.y  横移 m/s   （预留，当前策略未训练）
                                                      angular.z 偏航 rad/s （支持 0.6 ~ 1.0）
    发布  ~/odom           nav_msgs/Odometry        里程计（位姿 + **机体系**速度）
    发布  ~/scan           sensor_msgs/LaserScan    2D 激光，360 线，frame=base_link
    发布  ~/joint_states   sensor_msgs/JointState   12 个关节（给 robot_state_publisher）
    发布  ~/tf             tf2                      odom -> base_link（可用 publish_tf 关掉）

    frame 约定：world(父) / odom / base_link
                laser 数据直接发布在 base_link 系里（安装位已在内部补偿，
                默认机体前上方 0.19/0.00/0.20 m），**不需要额外的 laser_frame**

================================================================================
★ 三个必须知道的"坑"（都是实测踩出来的，写在最前面免得别人再踩）

  1) **指令有死区，别用太小的速度。**
     策略只在 0.3 m/s 以上才跟得动：

         指令 0.09 m/s -> 跟踪  1%     指令 0.30 m/s -> 跟踪 82%
         指令 0.15 m/s -> 跟踪  9%     指令 0.50 m/s -> 跟踪 96%
                                       指令 1.00 m/s -> 跟踪 99%

     同样的死区也存在于**偏航角速度**上：

         0.25 rad/s -> 42%     0.60 rad/s -> 86%     1.00 rad/s -> 92%

     所以 Nav2 侧的两个参数**必须**抬到工作区间，否则会出现
     "机器人站着一步不走"或"原地转半天转不过来"：
         controller_server.min_speed_xy    <= 建议 >= 0.28   （不能是 0.01）
         controller_server.min_speed_theta <= 建议 >= 0.45   （不能是 0.0）
         并且两者都要满足 min_* <= acc_lim_* / controller_frequency
     这是本项目排查最久的一类问题：参数看着都"合理"，只是落在死区里。

  2) **必须用墙钟时间。** 本节点用墙钟（`use_sim_time=false`）。
     如果你的栈设了 `use_sim_time=true` 又没人发 `/clock`，
     所有基于时间的逻辑（等待、超时、settle）都会失效 ——
     表现为"状态机停住不动"，非常难查。
     若要混用，请让 Gazebo/其他仿真器发布 `/clock`，并把本节点的
     `use_sim_time` 一起设成 true。

  3) **环境变量 ROS_LOCALHOST_ONLY=1（本机网卡异常时必需）。**
     如果本机有 DOWN/NO-CARRIER 的网卡，DDS 会去绑一个不通的接口，
     导致**进程间发现失败**：launch 侧一切正常，但某些节点收不到任何消息。
     多个终端一起用的时候务必都导出这个变量。

================================================================================
策略说明

  · 网络：MLP 48 -> 512 -> 256 -> 128 -> 24，激活 **swish**，
          最后过 **tanh**（brax 的 tanh_normal 策略头，取前 12 维作均值）。
  · 观测 48 维，**顺序与 scale 都固定**（改错不会崩，只会让机器人变傻）：
        [0:3]   机体系线速度（原始值，不打尺度）
        [3:6]   机体系角速度（原始值）
        [6:9]   投影重力（直立时应为 [0,0,-1]）
        [9:21]  关节角 - default_pose
        [21:33] 关节角速度
        [33:45] 上一步动作
        [45:48] 速度指令 [vx, vy, wz]（原始值）
  · 进网络前必须做**观测归一化** (x - mean) / std，统计量在
    config/go1_obs_normalizer.npz 里。漏掉这一步误差从 1e-7 涨到 0.43。
  · 控制：MuJoCo 内置位置执行器，ctrl = action * 0.5 + default_pose，
          50 Hz 策略（每周期 5 次 0.004 s 物理步）。
  · 已验证：本文件的 NumPy 前向与训练框架（MJX）输出**逐位一致（误差 1.2e-07）**。

参数：见 config/locomotion.yaml（全部可用 ros2 param 覆盖）
"""

import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, LaserScan

# ---------------- 训练侧常量（不要随意改：与权重一一对应）----------------
JOINTS = ["FR_hip", "FR_thigh", "FR_calf",
          "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf",
          "RL_hip", "RL_thigh", "RL_calf"]
# default_pose 会优先从模型的 home 关键帧读取（与训练严格一致），
# 这里的常量只是模型里没有关键帧时的兜底。
DEFAULT_POSE_FALLBACK = np.array(
    [0.1, 0.9, -1.8, -0.1, 0.9, -1.8,
     0.1, 0.9, -1.8, -0.1, 0.9, -1.8], dtype=np.float32)
KP_DEFAULT, KD_DEFAULT, ACTION_SCALE_DEFAULT = 35.0, 0.5, 0.5
SIM_DT_DEFAULT, N_SUBSTEPS_DEFAULT = 0.004, 5


class Go1Locomotion(Node):
    def __init__(self):
        super().__init__("go1_locomotion")

        # ---------------- 参数 ----------------
        def P(name, default):
            return self.declare_parameter(name, default).value

        share = self._share_dir()
        self.scene = P("scene", os.path.join(share, "config", "go1_corridor.mjb"))
        self.policy_path = P("policy_weights",
                             os.path.join(share, "config", "go1_policy_weights.npz"))
        self.norm_path = P("obs_normalizer",
                           os.path.join(share, "config", "go1_obs_normalizer.npz"))
        self.cmd_topic = P("cmd_vel_topic", "cmd_vel")
        self.odom_topic = P("odom_topic", "odom")
        self.scan_topic = P("scan_topic", "scan")
        self.js_topic = P("joint_states_topic", "joint_states")
        self.odom_frame = P("odom_frame", "odom")
        self.base_frame = P("base_frame", "base_link")
        self.publish_tf = P("publish_tf", True)
        self.cmd_timeout = P("cmd_timeout", 0.5)
        self.max_vx = P("max_linear_x", 1.0)
        self.max_wz = P("max_angular_z", 1.2)
        self.n_rays = P("laser_samples", 360)
        self.scan_rate = max(1.0, float(P("scan_rate", 10.0)))
        self.scan_period = 1.0 / self.scan_rate
        self.last_scan_t = 0.0
        self.laser_off = np.array([P("laser_x", 0.19), P("laser_y", 0.0),
                                   P("laser_z", 0.20)], dtype=np.float64)
        self.laser_range = P("laser_range", 10.0)
        self.laser_min_range = P("laser_min_range", 0.20)
        self.sim_dt = P("sim_dt", SIM_DT_DEFAULT)
        self.n_substeps = P("substeps", N_SUBSTEPS_DEFAULT)
        self.kp = P("kp", KP_DEFAULT)
        self.kd = P("kd", KD_DEFAULT)
        self.action_scale = P("action_scale", ACTION_SCALE_DEFAULT)
        self.reset_on_start = P("reset_on_start", True)
        self.enable_viewer = P("mujoco_viewer", False)
        self.viewer_follow = P("viewer_follow", True)

        # ---------------- 模型 ----------------
        import mujoco
        self.mujoco = mujoco
        if not os.path.exists(self.scene):
            raise SystemExit(f"找不到模型文件: {self.scene}")
        if self.scene.endswith(".mjb"):
            self.model = mujoco.MjModel.from_binary_path(self.scene)
        else:
            self.model = mujoco.MjModel.from_xml_path(self.scene)
        # 训练侧超参，逐项对齐（改错不报错，只会让策略变傻）
        self.model.opt.timestep = self.sim_dt
        self.model.opt.ccd_iterations = 20
        self.model.dof_damping[6:] = self.kd
        self.model.actuator_gainprm[:, 0] = self.kp
        self.model.actuator_biasprm[:, 1] = -self.kp
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        if self.enable_viewer:
            # Keep the MuJoCo viewer attached to this exact model/data pair so
            # the displayed robot is driven by the same policy as the ROS topics.
            try:
                from mujoco.viewer import launch_passive
                self.viewer = launch_passive(self.model, self.data)
                if self.viewer_follow:
                    # Chase camera on the trunk; drag/zoom still works in the window.
                    with self.viewer.lock():
                        cam = self.viewer.cam
                        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        cam.trackbodyid = mujoco.mj_name2id(
                            self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
                        cam.distance, cam.elevation, cam.azimuth = 4.0, -22.0, 200.0
            except Exception as exc:
                raise SystemExit(f"无法启动 MuJoCo 可视化窗口: {exc}") from exc

        # ---------------- 策略 ----------------
        self.layers, self.mean, self.std, self.eps = self._load_policy()
        self.default_pose = self._default_pose()

        self.qadr, self.dadr = [], []
        for j in JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j + "_joint")
            if jid < 0:
                raise SystemExit(f"模型里找不到关节 {j}_joint")
            self.qadr.append(self.model.jnt_qposadr[jid])
            self.dadr.append(self.model.jnt_dofadr[jid])

        self._reset()
        self.cmd = np.zeros(3, dtype=np.float32)
        self.last_cmd_t = time.time()
        self.last_action = np.zeros(12, dtype=np.float32)
        self.n_steps = 0

        # ---------------- ROS 接口 ----------------
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pub_scan = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.pub_js = self.create_publisher(JointState, self.js_topic, 10)
        if self.publish_tf:
            from tf2_ros import TransformBroadcaster
            self.tf_bc = TransformBroadcaster(self)
        self.create_timer(self.sim_dt * self.n_substeps, self._on_timer)

        self.get_logger().info(
            f"go1_locomotion 已就绪 | 模型 {os.path.basename(self.scene)} | "
            f"策略 {len(self.layers)} 层 | 控制 {1.0/(self.sim_dt*self.n_substeps):.0f} Hz | "
            f"激光 {self.n_rays} 线 | 指令 {self.cmd_topic} -> 里程计 {self.odom_topic}")

    # ------------------------------------------------------------------
    @staticmethod
    def _share_dir():
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory("go1_mujoco_locomotion")
        except Exception:
            return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def _load_policy(self):
        if not os.path.exists(self.policy_path):
            raise SystemExit(f"找不到策略权重: {self.policy_path}")
        if not os.path.exists(self.norm_path):
            raise SystemExit(
                f"找不到观测归一化统计量: {self.norm_path}\n"
                f"★ 这个文件不能省：漏掉归一化会让策略输出误差从 1e-7 涨到 0.43，"
                f"机器人会站不稳。")
        d = np.load(self.policy_path)
        idx = sorted(set(k.split(".")[0] for k in d.files), key=int)
        # 注意：kernel 是 (in_features, out_features) 行主序，前向用 h @ kernel，
        #       **不能转置**（PyTorch 是 (out,in)，很容易条件反射写错）
        layers = [(d[f"{i}.kernel"].astype(np.float32),
                   d[f"{i}.bias"].astype(np.float32)) for i in idx]
        nz = np.load(self.norm_path)
        return layers, nz["mean"].astype(np.float32), \
            nz["std"].astype(np.float32), np.float32(nz["std_eps"])

    def _default_pose(self):
        """优先取模型 home 关键帧的关节角（与训练侧 _default_pose 同源）。"""
        kf = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_KEY, "home")
        if kf >= 0:
            return np.array(self.model.key_qpos[kf][7:], dtype=np.float32)
        return DEFAULT_POSE_FALLBACK

    def _reset(self):
        kf = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_KEY, "home")
        if kf >= 0:
            self.data.qpos[:] = self.model.key_qpos[kf]
        else:
            self.data.qpos[:] = 0.0
            self.data.qpos[2] = 0.278
            self.data.qpos[3] = 1.0
            for i, a in enumerate(self.qadr):
                self.data.qpos[a] = DEFAULT_POSE_FALLBACK[i]
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = self.data.qpos[7:]
        self.mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    def _on_cmd(self, msg):
        self.cmd = np.array([float(np.clip(msg.linear.x, -self.max_vx, self.max_vx)),
                             float(np.clip(msg.linear.y, -self.max_vx, self.max_vx)),
                             float(np.clip(msg.angular.z, -self.max_wz, self.max_wz))],
                            dtype=np.float32)
        self.last_cmd_t = time.time()

    def _policy(self, obs):
        h = ((obs - self.mean) / (self.std + self.eps)).reshape(1, -1).astype(np.float32)
        for i, (k, b) in enumerate(self.layers):
            h = h @ k + b
            if i < len(self.layers) - 1:
                h = h * (1.0 / (1.0 + np.exp(-h)))     # swish
        return np.tanh(h.squeeze()[:12]).astype(np.float32)   # tanh_normal 取均值

    def _obs(self):
        d = self.data
        w, x, y, z = d.qpos[3:7]
        Rinv = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y + z*w), 2*(x*z - y*w)],
            [2*(x*y - z*w), 1 - 2*(x*x + z*z), 2*(y*z + x*w)],
            [2*(x*z + y*w), 2*(y*z - x*w), 1 - 2*(x*x + y*y)]], dtype=np.float32)
        obs = np.zeros(48, dtype=np.float32)
        obs[0:3] = Rinv @ d.qvel[0:3].astype(np.float32)
        obs[3:6] = d.qvel[3:6].astype(np.float32)
        obs[6:9] = np.array([2*(w*y - x*z), -2*(y*z + w*x),
                             2*(x*x + y*y) - 1], dtype=np.float32)
        obs[9:21] = np.array([d.qpos[a] for a in self.qadr], dtype=np.float32) - self.default_pose
        obs[21:33] = np.array([d.qvel[a] for a in self.dadr], dtype=np.float32)
        obs[33:45] = self.last_action
        obs[45:48] = self.cmd
        return obs

    def _on_timer(self):
        mj, m, d = self.mujoco, self.model, self.data
        # 指令超时归零：上层若挂掉，机器人会自己停下（安全兜底）
        if time.time() - self.last_cmd_t > self.cmd_timeout:
            self.cmd[:] = 0.0
        self.last_action = self._policy(self._obs())
        if self.viewer is not None:
            with self.viewer.lock():
                d.ctrl[:] = self.last_action * self.action_scale + self.default_pose
                for _ in range(self.n_substeps):
                    mj.mj_step(m, d)
            if self.viewer.is_running():
                self.viewer.sync()
        else:
            d.ctrl[:] = self.last_action * self.action_scale + self.default_pose
            for _ in range(self.n_substeps):
                mj.mj_step(m, d)
        self.n_steps += 1
        self._publish(publish_scan=(time.monotonic() - self.last_scan_t >= self.scan_period))

    # ------------------------------------------------------------------
    def _scan(self):
        mj, m, d = self.mujoco, self.model, self.data
        w, qx, qy, qz = d.qpos[3:7]
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*w), 2*(qx*qz + qy*w)],
            [2*(qx*qy + qz*w), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*w)],
            [2*(qx*qz - qy*w), 2*(qy*qz + qx*w), 1 - 2*(qx*qx + qy*qy)]])
        origin = np.array([d.qpos[0], d.qpos[1], d.qpos[2]]) + R @ self.laser_off
        yaw = math.atan2(2*(w*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        ranges = np.full(self.n_rays, self.laser_range, dtype=np.float32)
        gid = np.zeros(1, dtype=np.int32)
        for i in range(self.n_rays):
            ang = yaw + (-math.pi + 2*math.pi*i/self.n_rays)
            dist = mj.mj_ray(m, d, origin,
                             np.array([math.cos(ang), math.sin(ang), 0.0]),
                             None, 1, -1, gid)
            if dist > 0:
                ranges[i] = min(dist, self.laser_range)
        return ranges

    def _publish(self, publish_scan=True):
        d = self.data
        # Ray casting is the most expensive part of the ROS update. Stamp the
        # outgoing data after it finishes so AMCL never receives a scan older
        # than the matching odom->base transform when the host is under load.
        scan_ranges = self._scan() if publish_scan else None
        now = self.get_clock().now().to_msg()
        w, qx, qy, qz = d.qpos[3:7]

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(d.qpos[0])
        od.pose.pose.position.y = float(d.qpos[1])
        od.pose.pose.position.z = float(d.qpos[2])
        od.pose.pose.orientation.x = float(qx)
        od.pose.pose.orientation.y = float(qy)
        od.pose.pose.orientation.z = float(qz)
        od.pose.pose.orientation.w = float(w)
        # ★ twist 按**机体系**发布（ROS 约定）：Nav2 的局部规划按机体系理解它。
        #   用世界系会得到"机器人反向走"这类完全错误的结论（本工程踩过）。
        Rinv = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy + qz*w), 2*(qx*qz - qy*w)],
            [2*(qx*qy - qz*w), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz + qx*w)],
            [2*(qx*qz + qy*w), 2*(qy*qz - qx*w), 1 - 2*(qx*qx + qy*qy)]])
        lv = Rinv @ np.array(d.qvel[0:3])
        od.twist.twist.linear.x = float(lv[0])
        od.twist.twist.linear.y = float(lv[1])
        od.twist.twist.linear.z = float(lv[2])
        od.twist.twist.angular.x = float(d.qvel[3])
        od.twist.twist.angular.y = float(d.qvel[4])
        od.twist.twist.angular.z = float(d.qvel[5])
        self.pub_odom.publish(od)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(d.qpos[0])
            t.transform.translation.y = float(d.qpos[1])
            t.transform.translation.z = float(d.qpos[2])
            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(w)
            self.tf_bc.sendTransform(t)

        if scan_ranges is not None:
            sc = LaserScan()
            sc.header.stamp = now
            sc.header.frame_id = self.base_frame     # 安装位已在内部补偿
            sc.angle_min = -math.pi
            sc.angle_max = math.pi
            sc.angle_increment = 2 * math.pi / self.n_rays
            sc.range_min = float(self.laser_min_range)
            sc.range_max = float(self.laser_range)
            sc.ranges = [float(r) for r in scan_ranges]
            self.pub_scan.publish(sc)
            self.last_scan_t = time.monotonic()

        js = JointState()
        js.header.stamp = now
        js.name = [j + "_joint" for j in JOINTS]
        js.position = [float(d.qpos[a]) for a in self.qadr]
        js.velocity = [float(d.qvel[a]) for a in self.dadr]
        self.pub_js.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = Go1Locomotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
