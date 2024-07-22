#! /bin/sh

# Launch one drone and bridge

cd ~/PX4-Autopilot

PX4_SYS_AUTOSTART=4012 PX4_SIM_MODEL=tucan PX4_GZ_WORLD=tucan ./build/px4_sitl_default/bin/px4 -i 0