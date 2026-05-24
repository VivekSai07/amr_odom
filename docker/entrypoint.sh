#!/bin/bash
set -e
. /opt/ros/humble/setup.bash
. /ws/install/setup.bash
exec "$@"
