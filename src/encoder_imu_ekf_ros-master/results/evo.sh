#! /bin/bash
cd $2 && evo_traj bag $1 /current_pose --save_as_tum
evo_traj bag $1 /ndt_pose --save_as_tum
evo_traj bag $1 /wheel_odom_pose --save_as_tum
evo_traj tum wheel_odom_pose.tum ndt_pose.tum --ref=current_pose.tum -p --plot_mode=xy
