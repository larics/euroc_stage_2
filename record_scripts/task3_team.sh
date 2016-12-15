#!/bin/bash

TASK_NUMBER='task3'
source current_team_name.txt

echo "================================================="
echo "Start Recording $TASK_NUMBER for Team: " $TEAM_NAME
echo "on TEAM"
echo "================================================="
echo ""

mkdir -p "$HOME/results/bags"

rosbag record \
/$TEAM_NAME/imu \
/$TEAM_NAME/half_score_vicon_pose \
/$TEAM_NAME/num_subbed_to_half_score \
/$TEAM_NAME/do_not_subscribe/forbidden_vicon_odometry \
/$TEAM_NAME/do_not_subscribe/forbidden_vicon_transform \
/$TEAM_NAME/do_not_subscribe/num_odometry_subscribers \
/$TEAM_NAME/do_not_subscribe/num_transform_subscribers \
-o "$HOME/results/bags/${TEAM_NAME}_${TASK_NUMBER}_team"
