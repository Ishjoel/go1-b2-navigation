#!/usr/bin/env python3
"""Finish Nav2 bringup if a lifecycle manager stalls on a lost service reply.

With Fast DDS, a lifecycle ``change_state`` response sent before the client's
reply reader is matched can be dropped. nav2_lifecycle_manager then waits on
that request forever: AMCL never activates, no map->odom transform appears and
the navigation bringup aborts. After a grace period this node checks every
managed node and drives any that are still unconfigured/inactive to active.
"""
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

LOCALIZATION = ["map_server", "amcl"]
NAVIGATION = ["controller_server", "smoother_server", "planner_server", "route_server",
              "behavior_server", "velocity_smoother", "collision_monitor", "bt_navigator",
              "waypoint_follower", "docking_server"]


class LifecycleGuard(Node):
    def __init__(self):
        super().__init__("lifecycle_guard")
        self.grace = self.declare_parameter("grace_period", 20.0).value
        self.deadline = self.declare_parameter("give_up_after", 180.0).value
        self.clients_ = {}

    def call(self, node, srv_type, suffix, request, timeout=3.0):
        key = (node, suffix)
        if key not in self.clients_:
            self.clients_[key] = self.create_client(srv_type, f"/{node}/{suffix}")
        client = self.clients_[key]
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def state(self, node):
        result = self.call(node, GetState, "get_state", GetState.Request())
        return None if result is None else result.current_state.id

    def transition(self, node, transition_id, label):
        request = ChangeState.Request()
        request.transition.id = transition_id
        result = self.call(node, ChangeState, "change_state", request, timeout=10.0)
        self.get_logger().warn(f"{node}: {label} -> {'ok' if result and result.success else 'retry'}")

    def ensure_active(self, nodes):
        for node in nodes:
            state = self.state(node)
            if state == State.PRIMARY_STATE_ACTIVE:
                continue
            if state == State.PRIMARY_STATE_UNCONFIGURED:
                self.transition(node, Transition.TRANSITION_CONFIGURE, "configure")
                state = self.state(node)
            if state == State.PRIMARY_STATE_INACTIVE:
                self.transition(node, Transition.TRANSITION_ACTIVATE, "activate")
            # Later nodes depend on earlier ones (planner needs map->odom).
            return False
        return True

    def run(self):
        start = time.monotonic()
        while time.monotonic() - start < self.grace:
            rclpy.spin_once(self, timeout_sec=0.5)
        recovered = False
        while time.monotonic() - start < self.deadline:
            if self.ensure_active(LOCALIZATION) and self.ensure_active(NAVIGATION):
                if recovered:
                    self.get_logger().warn("Nav2 bringup completed by lifecycle_guard")
                else:
                    self.get_logger().info("Nav2 bringup healthy")
                return 0
            recovered = True
            time.sleep(1.0)
        self.get_logger().error("Nav2 nodes still not active; restart the demo")
        return 1


def main():
    rclpy.init(args=sys.argv)
    node = LifecycleGuard()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
