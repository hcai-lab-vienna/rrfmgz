#/bin/bash

for seed in lf0 lf1 lf2 lf3 lf4 lf5 lf6 lf7 lf8 lf9
do
	for val in '0' '1'
	do
		export RRFM_SEED="$seed"
		export RRFM_ALWAYS_RANDOM_ROTATION="$val"
		echo run with SEED="$seed" and ALWAYS_RANDOM_ROTATION="$val"
		sleep 5
		timeout 500s ros2 run rrfm rrfm_node
		sleep 5
		gz service -s /world/scout_lehrforst/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'
	done
done

