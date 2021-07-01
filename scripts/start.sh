#!/bin/bash
cd catkin_ws/src
catkin_create_pkg spawn_objects std_msgs rospy
cd spawn_objects
mkdir scripts
cd scripts
cp /RoboticsAcademy/exercises/pick_place/web-template/launch/model_manager.py /catkin_ws/src/spawn_objects/scripts/model_manager.py
cp /RoboticsAcademy/exercises/pick_place/web-template/launch/models_info.yaml /catkin_ws/src/spawn_objects/scripts/models_info.yaml
cd /
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000 &
python3.8 manager.py
