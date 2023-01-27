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

pose_sent = false;
pause_time = 2.2;
curr_time = tic;

human_pos_buf = zeros(3, 10);

load('goal_locs.mat');
pieces = ["red_1x2" "red_2x6" "red_1x8" "orange_1x2"];
human_goals = [];
for i=1:pieces.length
    human_goals = [human_goals goal_locs.(pieces(i)).pick.xyz];
end

while true
    while true
        flushdata(depthVid);
        [frameDataDepth, timeDataDepth, metaData] = getdata(depthVid);               
        metaDataDepth = metaData(framesPerTrig);
        if  sum(metaDataDepth.IsBodyTracked) > 0
            trackedBodies = find(metaDataDepth.IsBodyTracked);

            % Find number of Skeletons tracked.
            nBodies = length(trackedBodies);

            % Get the joint indices of the tracked bodies with respect to the color image.
            colorJointIndices = metaDataDepth.JointPositions(:, :, trackedBodies);

            % Overlay the skeleton on this RGB frame.
            x = -colorJointIndices(:,1);
            y = -colorJointIndices(:,2);
            z = colorJointIndices(:,3);
            for i=1:size(colorJointIndices, 1)
                pt = [x(i); y(i); z(i)];
                pt = Trans(1:3, 1:3)*pt+Trans(1:3, 4);
                x(i) = pt(1);
                y(i) = pt(2);
                z(i) = pt(3);
            end

            HuCap{1}.p=[x(4),x(4);y(4),y(4);z(4),z(4)];       %head-head
            HuCap{2}.p=[x(3),x(1);y(3),y(1);z(3),z(1)];       %shoulder center-hip center
            HuCap{5}.p=[x(5),x(6);y(5),y(6);z(5),z(6)];       %shoulder left-elbow left
            HuCap{6}.p=[x(6),x(7);y(6),y(7);z(6),z(7)];      %elbow left-wrist left
            HuCap{3}.p=[x(9),x(10);y(9),y(10);z(9),z(10)];    %shoulder right-elbow right
            HuCap{4}.p=[x(10),x(11);y(10),y(11);z(10),z(11)]; %elbow right-wrist right
            HuCap{9}.p=[x(13),x(14);y(13),y(14);z(13),z(14)]; %hip left-knee left
            HuCap{10}.p=[x(14),x(15);y(14),y(15);z(14),z(15)]; %knee left-ankle left
            HuCap{7}.p=[x(17),x(18);y(17),y(18);z(17),z(18)]; %hip right-knee right
            HuCap{8}.p=[x(18),x(19);y(18),y(19);z(18),z(19)]; %knee right-ankle right
            break;
        end
    end
    wrist_pos = HuCap{4}.p(:,2);
%     disp(wrist_pos);
    % save the human's current wrist position to buffer
    human_pos_buf = circshift(human_pos_buf, [0, -1]);
    human_pos_buf(:,end) = wrist_pos;

    [jpos, jvel, SSA_status, controller_status] = comm.getRobData;
    robot_pos = FK(robot, jpos);
    disp(robot_pos(1:3,4));
    
    % send pose command to the robot (only once), otherwise continue
    % waiting until pause_time has elapsed
    
    
end
%%
comm.endSTMO;
pause(0.5);
%%
stop(tg);
if MODE == 'CamerINTR'
    stop(depthVid);
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
% pause(pause_time);
end