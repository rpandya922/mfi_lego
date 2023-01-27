% This is the robot handover pipeline.
% Copyright (C) 2022  
% 
% Authors:
% Ruixuan Liu: ruixuanl@andrew.cmu.edu
% Rui Chen: ruic3@andrew.cmu.edu
% Changliu Liu : cliu6@andrew.cmu.edu
% 
% This program is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public License
% as published by the Free Software Foundation; either version 2
% of the License, or (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

init_pos = [1,1];

% CFS Parameters
traj_hz = 1; % Assume traj is 1hz.
resample_hz = 25;
assert(floor((1/traj_hz)/0.008/resample_hz) == ((1/traj_hz)/0.008/resample_hz));
u = init_pos;
human_pre_pos = init_pos;

%% begin
% replan_cnt = 1;
sequence_number = sequence_number + 1;
bootup = false;
STATE = "init_s"; STATE_last = "none";
goal_type = "regular"; goal_type_last = "none";

ovr = 0.1;

% move home
home_pos = [0.445; 27.224; -44.181+27.224; 2.789; -49.112; -4.930];
pause_time = 4;
move_to_goal(home_pos, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);
%% picking up pieces
sequence_number = sequence_number + 1;
pause_time = 2.5;
goal = [-6.986; 78.521; -32.095+78.521; 3.777; -59.466; 5.988];
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);

% grasp piece
sequence_number = sequence_number + 1;
pause_time = 4.2;
goal = [-7.083; 80.970; -33.265+80.970; 3.827; -58.292; 5.991];
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);

% rotate piece to remove
sequence_number = sequence_number + 1;
pause_time = 4.2;
goal = [-7.378; 75.194; -39.412+75.194; 2.663; -42.134; 6.764];
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);

%% move to home after picking
sequence_number = sequence_number + 1;
home_pos = [0.445; 27.224; -44.181+27.224; 2.789; -49.112; -4.930];
pause_time = 4;
move_to_goal(home_pos, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);

%% dropping off pieces
% hover over piece
sequence_number = sequence_number + 1;
pause_time = 2.5;
goal = [5.320; 59.500; -57.462+59.500; 0.193; -33.813; 83.595];
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);

% grasp piece
sequence_number = sequence_number + 1;
pause_time = 4.2;
goal = [5.317; 61.464; -58.792+61.464; 0.200; -32.483; 83.589];
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);

% rotate piece to remove
sequence_number = sequence_number + 1;
pause_time = 4.2;
goal = [9.331; 61.808; -57.902+61.808; 24.711; -38.851; 59.651];
move_to_goal(goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);

%%
% comm.endSTMO;
% stop(tg);

% positions
% above yellow piece: 3.954 47.839 -79.808 -1.187 -11.579 0
% grasping yellow piece: -3.948 51.496 -82.271 -1.502 -9.116 0

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