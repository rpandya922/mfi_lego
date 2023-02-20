traj_hz = 1;
resample_hz = 25;
ovr = 0.1;
pause_time = 2.05;
ROBOT = 'LRMate200iD7L';
load_dependencies;
robot = robot_property(ROBOT);
solver = ik_solver(robot);

% test home position
load('goal_locs.mat');
home_pose = FK(robot, deg2rad(goal_locs.red_2x6.drop.up));
home_pose(1:3,4) = home_pose(1:3,4) - robot.base;
% rotation matrix for arm downward
% rot = [1 0 0; 0 -1 0; 0 0 -1];
% rot = eul2rotm([0 0 pi]); % in ZYX convention
% rotation matrix for arm dropping red 1x8
% rot = [0 1 0; 1 0 0; 0 0 -1];
% rot = eul2rotm([pi/2 0 pi]);
% NOTE: to rotate end effector while keeping arm pointed straight down,
% rotate around Z-axis
% xyz for home
% xyz = [0.55; 0; -0.18];

% red 2x6 dropoff eul2rotm([-pi/2 0 pi]) [0.677 0.1644 -0.42]
% -x -> moves forwards (towards robot base)
% +y -> moves robot right (from across table POV)
% +z -> moves upwards
% z position = -0.438 -> grasping second layer piece
rot = eul2rotm([-pi/2 0 pi]); % ZYX
% above 2x6 1 down from top
% xyz = [0.671 0.166 -0.44];
% above 2x6 2 down from top
xyz = [0.679; 0.166; -0.438];
% approximate size of one lego spot: 0.008
% spot_size_x = 0.0083;
% spot_size_y = 0.008;
% 7 spots left 1 spot forward
% xyz(1) = xyz(1) - (1*spot_size_x);
% xyz(2) = xyz(2) - (7*spot_size_y);

% home_pose = eye(4);
% home_pose(1:3,1:3) = rot;
% home_pose(1:3,4) = xyz;
% 
% weights = [0.25 0.25 0.25 1 1 1];
% init_guess = [0; 0; 0; 0; 0; 0];
% 
% [configSol, solInfo] = solver('body6', home_pose, weights, init_guess);
% % need to add pi/2 to configSol(2) so it's the same as our initial config
% configSol(2) = configSol(2) + (pi/2);
configSol = solve_ik(xyz, rot, solver, comm);

sequence_number = sequence_number + 1;
goal = rad2deg(configSol);
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);

% trying to detach from piece
rot = eul2rotm([-pi/2 -0.25 pi]); % ZYX
xyz = xyz + [0; -0.03; 0.005];
configSol = solve_ik(xyz, rot, solver, comm);

sequence_number = sequence_number + 1;
goal = rad2deg(configSol);
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);

function configSol = solve_ik(xyz, rot, solver, comm)
pose = eye(4);
pose(1:3,1:3) = rot;
pose(1:3,4) = xyz;

weights = [0.25 0.25 0.25 1 1 1];
% init_guess = [0; 0; 0; 0; 0; 0];
[jpos, ~, ~, ~] = comm.getRobData;
init_guess = deg2rad(jpos);
init_guess(2) = init_guess(2) - (pi/2);

[configSol, ~] = solver('body6', pose, weights, init_guess);
% need to add pi/2 to configSol(2) so it's the same as our initial config
configSol(2) = configSol(2) + (pi/2);
end

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
