#!/bin/bash

SCRIPT=$(readlink -f $0)
BIN=`dirname "$SCRIPT"`

echo '******* Launching SUV Simulation in Progress *******'
rosclean purge -y
gnome-terminal --tab --title="gazebo" -- bash -i -c "exec roslaunch --disable-title suv_simulation suv.launch"
#rosservice call /gazebo/unpause_physics
sleep 10
echo '******* SUV Simulation Launched Successfully *******'
gnome-terminal --tab --title="autopilot" -- bash -c "exec sim_vehicle.py -v APMrover2 -f gazebo-rover -m --console -L Regatta --out udp:127.0.0.1:14551
; bash"
sleep 15
echo '******* Autopilot Launched Successfully *******'
gnome-terminal --tab --title="mavros" -- bash -c "exec roslaunch --disable-title suv_simulation apm.launch; exec bash" 
sleep 5
echo '******* Mavros Topics are Now Available *******'
gnome-terminal --tab --title="odom" -- bash -c "rosrun pollutant_marker rotate_odom_node; exec bash" 
sleep 5
echo '******* Robot Odom Topic is Now Available *******'
gnome-terminal --tab --title="pollutant" -- bash -c "history -s \"rosrun pollutant_marker pollutant_marker_node\" ;  rosrun pollutant_marker pollutant_marker_node; exec bash" 
sleep 2
echo '******* Pollutant Marker is Launched *******'
gnome-terminal --tab --title="rviz" -- bash -c "history -s \"rosrun rviz rviz -d ~/suv_ws/src/suv_simulation/rviz/rviz_config_cam.rviz\"; rosrun rviz rviz -d ~/suv_ws/src/suv_simulation/rviz/rviz_config_cam.rviz ; exec bash"
sleep 5
echo '******* RVIZ Launched Successfully *******'
gnome-terminal --tab --title="m_pserver" -- bash -c "cd ~/m-planner_ceri-sn && python3 -m http.server 8000
; exec bash"
sleep 5
echo '******* CERI SN M-Planner Server Launched Successfully *******'
gnome-terminal --tab --title="API" -- bash -c "cd ~/m-planner_ceri-sn/python && python3 flask_server.py 
; exec bash"
sleep 5

google-chrome http://localhost:8000 &

echo '******* CERI SN M-Planner-QgroundControl API Launched Successfully *******'
gnome-terminal --tab --title="qgroundcontrol" -- bash -c "~/QGroundControl.AppImage; exec bash"
sleep 20
echo '******* Groundcontrol Launched Successfully *******'

