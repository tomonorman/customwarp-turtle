#!/bin/sh -l

set -x

make install

Xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3

source /ws/install/setup.sh 
ros2 launch cam_test sim1.launch.py output_video:=/ws/output/rec.avi

