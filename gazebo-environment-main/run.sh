roslaunch vid_estimator vid_sim.launch & sleep 3;
roslaunch rotors_gazebo simulator.launch & sleep 5;
roslaunch ego_planner run_in_exp.launch & sleep 3;
wait;