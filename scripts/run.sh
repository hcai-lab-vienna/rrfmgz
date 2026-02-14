#/bin/bash

source install/setup.bash

export RRFM_TTL=600

for seed in "$@"
do
	for val in '0' '1'
	do
		export RRFM_SEED="$seed"
		export RRFM_ALWAYS_RANDOM_ROTATION="$val"
		echo run with: SEED="$seed" ALWAYS_RANDOM_ROTATION="$val" DURATION="$RRFM_TTL"
		timeout "$((RRFM_TTL + 60))" ros2 launch ros_gz_lehrforst_sim scout_bumper_headless.launch.xml > /dev/null &
		sleep 60
		timeout "$RRFM_TTL" ros2 run rrfm rrfm_node
		sleep 60
	done
done
