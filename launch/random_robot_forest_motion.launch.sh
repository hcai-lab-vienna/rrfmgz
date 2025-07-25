#!/bin/bash

# for i in {0..4}; do python3 src/force_bar_topic_merger.py $i & done

ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=config/bridges.yaml &

for i in {0..4}; do
    python3 python/force_bar_topic_merger.py $i &
done
gz topic -t '/cmd_vel' -m gz.msgs.Twist -p 'linear: {x: 0.5}, angular: {z: 0.00}'
python3 python/random_robot_forest_motion.py
kill $(jobs -p)