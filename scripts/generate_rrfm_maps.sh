#/bin/bash

# Generates multiple position maps of a robot driving around a fixed 3D forest map and pumping into things.
# When robot collides:
# 1.   it drives back for a fixed amount of time
# 2.1. it either turns in a direction based on the segment which first detect the collision (center segment always random) [Multi-Segment approach]
# 2.2. or it always turns in a random direction to simulate a single segment bumper [Single-Segment approach]
# 3.   it turns in the direction from point 2. for a random duration, the duration should roughly be between 60 and 180 degree.
#
# This script takes a list of seeds (strings separated by spaces) as argument (at leas one must be provided).
# For each of those seeds both [Multi-Segment approach] and [Single-Segment approach] are run.
# The seed influences the behavior of points: 2.1. (center segment only), 2.2. and 3.

if [ $# -eq 0 ]; then
    echo "Usage: $0 SEED1 SEED2 SEED3 ..."
    exit 1
fi

source install/setup.bash

# duration of each individual simulation
export RRFM_TTL=600

for seed in "$@"
do
	for val in '0' '1' # '[Multi-Segment approach]' '[Single-Segment approach]'
	do
		export RRFM_SEED="$seed"
		export RRFM_ALWAYS_RANDOM_ROTATION="$val"
		echo run with: SEED="$seed" ALWAYS_RANDOM_ROTATION="$val" DURATION="$RRFM_TTL"
		# using timeout is more reliable then the builtin gazebo restart function
		# but this requires the simulation to run close to real time,
		# since the simulation duration is done by the shell
		timeout "$((RRFM_TTL + 60))" ros2 launch ros_gz_lehrforst_sim scout_bumper_headless.launch.xml > /dev/null &
		sleep 60
		timeout "$RRFM_TTL" ros2 run rrfm rrfm_node
		sleep 60
	done
done
