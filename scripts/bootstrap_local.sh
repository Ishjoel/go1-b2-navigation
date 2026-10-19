#!/usr/bin/env bash
# Use official packages in a workspace-local prefix when sudo is unavailable.
set -e
cd "$(dirname "$0")/.."
mkdir -p .deps/debs
cd .deps/debs
apt-get download ros-jazzy-gz-ros2-control ros-jazzy-joint-trajectory-controller ros-jazzy-control-msgs ros-jazzy-realtime-tools ros-jazzy-diagnostic-updater
cd ../..
for package_file in .deps/debs/*.deb; do dpkg-deb -x "$package_file" .deps/root; done

mkdir -p .deps/diagnostic/lib
mv .deps/root/opt/ros/jazzy/lib/libdiagnostic_updater.so .deps/diagnostic/lib/
