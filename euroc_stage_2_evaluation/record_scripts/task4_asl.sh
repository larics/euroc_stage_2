#!/bin/bash

TASK_NUMBER='task4'
source current_team_name.txt

echo "================================================="
echo "Start Recording $TASK_NUMBER for Team: " $TEAM_NAME
echo "on ASL"
echo "================================================="
echo ""

mkdir -p "$HOME/results/bags"

rosbag record \
/saver_constraints_violated \
/ground_truth_octree/octomap_full \
/${TEAM_NAME}/vrpn_client/estimated_transform \
/${TEAM_NAME}/vrpn_client/raw_transform \
/${TEAM_NAME}/vrpn_client/estimated_odometry \
/pipes_thing/vrpn_client/estimated_transform \
/pipes_thing/vrpn_client/raw_transform \
/scaffold_A/vrpn_client/estimated_transform \
/scaffold_A/vrpn_client/raw_transform \
/scaffold_B/vrpn_client/estimated_transform \
/scaffold_B/vrpn_client/raw_transform \
-o "$HOME/results/bags/${TEAM_NAME}_${TASK_NUMBER}_asl"
