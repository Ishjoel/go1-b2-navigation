#!/usr/bin/env bash
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
LOCAL_ROS="$WORKSPACE_ROOT/.deps/root/opt/ros/jazzy"
if [ -d "$LOCAL_ROS" ]; then
 export AMENT_PREFIX_PATH="$LOCAL_ROS:$AMENT_PREFIX_PATH"
 export LD_LIBRARY_PATH="$LOCAL_ROS/lib:$LD_LIBRARY_PATH"
 export GZ_SIM_SYSTEM_PLUGIN_PATH="$LOCAL_ROS/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
 export PYTHONPATH="$LOCAL_ROS/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi
[ ! -f "$WORKSPACE_ROOT/install/setup.bash" ] || source "$WORKSPACE_ROOT/install/setup.bash"
export ROS_LOG_DIR="$WORKSPACE_ROOT/log/ros"
mkdir -p "$ROS_LOG_DIR"
# Keep the Go1 MuJoCo/Nav2 demo isolated from older ROS sessions on the host.
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

export WORKSHOP_ROOT="$WORKSPACE_ROOT"
