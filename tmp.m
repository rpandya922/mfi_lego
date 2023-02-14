ROBOT = 'LRMate200iD7L';
load_dependencies;
robot = robot_property(ROBOT);
solver = ik_solver(robot);

% test home position
load('goal_locs.mat');
home_pose = FK(robot, deg2rad(goal_locs.neutral));
home_pose(1:3,4) = home_pose(1:3,4) - robot.base;

weights = [0.25 0.25 0.25 1 1 1];
init_guess = [0; 0; 0; 0; 0; 0];

[configSol, solInfo] = solver('body6', home_pose, weights, init_guess);
% need to add pi/2 to configSol(2) so it's the same as our initial config
configSol(2) = configSol(2) + (pi/2);

sequence_number = sequence_number + 1;
goal = rad2deg(configSol);
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);

function move_to_goal(goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time, slow)
[jpos, ~, ~, ~] = comm.getRobData;
comm.enableSSA(enb_ssa);
if slow
    ref_traj = [jpos'+(goal'-jpos')/4;
                jpos'+(goal'-jpos')/4*2;
                jpos'+(goal'-jpos')/4*3;
                goal'];
else
    ref_traj = [jpos'+(goal'-jpos')/2;
                goal'];
end
comm.drvJntTraj(ref_traj, traj_hz, resample_hz, ovr, replan_cnt);
pause(pause_time);
end
