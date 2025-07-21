#!/bin/bash

# for i in {0..4}; do python3 src/force_bar_topic_merger.py $i & done

ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=config/bridges.yaml &

python3 python/force_bar_topic_merger.py 0 &
python3 python/force_bar_topic_merger.py 1 &
python3 python/force_bar_topic_merger.py 2 &
python3 python/force_bar_topic_merger.py 3 &
python3 python/force_bar_topic_merger.py 4 &
python3 python/random_robot_forest_motion.py
kill $(jobs -p)