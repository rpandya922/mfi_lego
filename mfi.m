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

clear all;
clc;
close all;
ROBOT = 'LRMate200iD7L';
MODE = 'HumanINTR';%'CamerINTR'; %'CamerINTR'; % HumanINTR for mouse control; HuRobINTR: perception using camera.W
USE_ROBOT = 1; % True: real robot involved. False: only target machine. No robot needed.
enbSSA = 0; % 1: enable SSA. 0: disable SSA.
tasks = load('task.txt')';
USE_GRIPPER = 0;
USE_FTS = 0;
load_dependencies;
num_tasks = size(tasks, 2);
task_id = 1;
% task_id_last = -1;
bootup = true;
robot = robot_property(ROBOT);

HuCap=human_cap(6, [1,1]);
pre_HuCap=HuCap;
vel_t=1000;

% Setup sound
correct_pos_sound_filename = 'correct_pos_sound.wav';
[correct_pos_sound,correct_pos_sound_Fs] = audioread(correct_pos_sound_filename);
samples = [1,floor(0.3*correct_pos_sound_Fs)];
[correct_pos_sound,correct_pos_sound_Fs] = audioread(correct_pos_sound_filename,samples);

% Setup Gripper
if USE_GRIPPER
    setup_gripper;
end

% Setup FTS
if USE_FTS
    serial = py.importlib.import_module('serial'); 
    py.importlib.reload(serial);
    struct=py.importlib.import_module('struct');
    ser=connectFTS(serial);
    init(ser,struct);
end

% Setup Target PC
tg = slrealtime('fanuc_speedgoat');
if USE_ROBOT
    load(tg, 'icl_control');
else
    load(tg, 'icl_control_sim');
end
start(tg);
comm=SLRTComm;
comm.timeout=20;
comm.startSTMO;
pause(0.5);
comm.setRobotProperty(robot.nlink,robot.DH,robot.cap,robot.base,robot.ssa_margin);
comm.setHumanProperty(HuCap,pre_HuCap,vel_t);
pause(0.1);
comm.enableSSA(enbSSA);
[jpos, jvel, SSA_status, controller_status] = comm.getRobData;
% draw_robot;
% pause(0.2);

init_pos = [1,1];

% CFS Parameters
traj_hz = 1; % Assume traj is 1hz.
resample_hz = 25;
assert(floor((1/traj_hz)/0.008/resample_hz) == ((1/traj_hz)/0.008/resample_hz));
u = init_pos;
human_pre_pos = init_pos;

%% begin
replan_cnt = 1;
bootup = false;
STATE = "init_s"; STATE_last = "none";
goal_type = "regular"; goal_type_last = "none";
% goal = zeros(6, 1);
% vel_timer = tic;
% wrist_trigger_cnt = 0;
% home_pos = [0;0;0;0;-90;0];
% 
% % first move the am to home_pos
% ref_traj = [jpos'+(home_pos'-jpos')/5;
%             jpos'+(home_pos'-jpos')/5*2;
%             jpos'+(home_pos'-jpos')/5*3;
%             jpos'+(home_pos'-jpos')/5*4;
%             home_pos'];
% ovr = 0.1;
% comm.drvJntTraj(ref_traj, traj_hz, resample_hz, ovr, replan_cnt);
% pause(5);
%%
enb_ssa = 0;
ovr = 0.1;
j3_offset = 47.842; % commanded -20, shows -67.842
pause_time = 5;

% above_yellow_goal = [3.954; 47.839; -79.808+j3_offset; -1.187; -11.579; 0];
% yellow_goal = [-3.948; 51.496; -82.271+j3_offset; -1.502; -9.116; 0];

% replan_cnt = replan_cnt + 1;
% j3_offset = 29.439; % j2/j3 interaction -29.439
j3_offset = 46.415; % commanded -20, shows -66.415
above_red_goal = [15.719; 46.412; -75.852+j3_offset; 2.776; -12.806; -16.266];
move_to_goal(above_red_goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time); 

replan_cnt = replan_cnt + 1;
j3_offset = 48.388; % commanded -20, shows -68.388
% red_goal = [15.538; 48.388; -76.617+j3_offset; 0.701; -14.079; -14.079]; % interaction -28.228
% red_goal = [15.538; 48.388; -20; 0.701; -14.079; -14.079]; % interaction -28.228
red_goal = [15.566; 48.472; -76.619+48.472; 0.651; -13.927; -14.057];
move_to_goal(red_goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time); 

replan_cnt = replan_cnt + 1;
% j3_offset = 48.388;
% grasp_goal = [15.538; 48.388; -76.617+j3_offset; 0.701; -10.079; -14.079];
grasp_goal = [15.613; 46.916; -75.888+46.916; 2.518; -13.141; -15.913];
move_to_goal(grasp_goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time); 

% traj_hz = 2; resample_hz = 12.5;
% traj_hz = 5;
replan_cnt = replan_cnt + 1;
j3_offset = 49.974; % commanded -20, shows -69.974
above_black_goal = [10.782; 49.974; -66.458+j3_offset; -2.243; -25.467; -6.573];
move_to_goal(above_black_goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time); 

replan_cnt = replan_cnt + 1;
% j3 offset ends up being joint position of j2
% -j2 + interact = j3 (on pad)
drop_red_goal = [10.814; 52.246; -68.242+52.246; -2.401; -23.820; -8.716];
move_to_goal(drop_red_goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time); 

replan_cnt = replan_cnt + 1;
pause_time = 5.5;
% % remove_red_goal = [10.814; 52.246; -68.242+52.246; -2.401; -28.000; -8.716];
% remove_red_goal = [10.741; 50.842; -66.712+50.842; -2.838; -27.055; -8.278];
% remove_red_goal = [10.459; 51.633; -64.923+51.633; -3.352; -33.440; -7.593];
remove_red_goal = [10.715; 51.260; -66.816+51.260; -3.093; -27.756; -8.028];
move_to_goal(remove_red_goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time); 

%%
comm.endSTMO;
stop(tg);

% positions
% above yellow piece: 3.954 47.839 -79.808 -1.187 -11.579 0
% grasping yellow piece: -3.948 51.496 -82.271 -1.502 -9.116 0

function move_to_goal(goal, enb_ssa, comm, replan_cnt, traj_hz, resample_hz, ovr, pause_time)
[jpos, ~, ~, ~] = comm.getRobData;
comm.enableSSA(enb_ssa);
ref_traj = [jpos'+(goal'-jpos')/5;
            jpos'+(goal'-jpos')/5*2;
            jpos'+(goal'-jpos')/5*3;
            jpos'+(goal'-jpos')/5*4;
            goal'];
comm.drvJntTraj(ref_traj, traj_hz, resample_hz, ovr, replan_cnt);
pause(pause_time);
end