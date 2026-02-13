#/bin/bash

source install/setup.bash

export RRFM_TTL=600

for seed in alf0 alf1 alf2 alf3 alf4 alf5 alf6 alf7 alf8 alf9 blf0 blf1 blf2 blf3 blf4 blf5 blf6 blf7 blf8 blf9 clf0 clf1 clf2 clf3 clf4 clf5 clf6 clf7 clf8 clf9 dlf0 dlf1 dlf2 dlf3 dlf4 dlf5 dlf6 dlf7 dlf8 dlf9 flf0 flf1 flf2 flf3 flf4 flf5 flf6 flf7 flf8 flf9
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
