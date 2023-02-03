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

ROB_GOAL_MODE = 'proactive'; % 'naive', 'responsive'

% move home
% home_pos = [0.445; 27.224; -44.181+27.224; 2.789; -49.112; -4.930];
% pause_time = 4;
% move_to_goal(home_pos, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);

pose_sent = false;
pause_time = 2.2;
curr_time = tic;

human_pos_buf = zeros(3, 10);

load('goal_locs.mat');
pieces = ["red_1x2" "red_2x6" "red_1x8" "orange_1x2"];
human_goals = [];
for i=1:length(pieces)
    human_goals = [human_goals goal_locs.(pieces(i)).pick.xyz_h];
end

h_pieces = ["green_1x4" "blue_1x4" "pink_1x4" "red_1x8" "red_2x6"...
    "yellow_1x6_b" "yellow_1x6_t"];
h_pieces_active = logical([1 0 0 1 1 0 1]);
r_pieces = ["red_1x2" "orange_1x2" "red_2x6" "red_1x8"];
r_pieces_active = logical([1 1 1 1]);

STATE = 'SENSE'; % 'MOVE'
prev_h_pred = -1;
prev_prev_h_pred = -1;
rob_cmd_sent = false;

loop_counter = 0;
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
    switch STATE
        case 'SENSE'
            wrist_pos = HuCap{4}.p(:,2);
%             disp("human");
%             disp(wrist_pos);
            % save the human's current wrist position to buffer
            human_pos_buf = circshift(human_pos_buf, [0, -1]);
            human_pos_buf(:,end) = wrist_pos;

            [jpos, jvel, SSA_status, controller_status] = comm.getRobData;
            robot_pos = FK(robot, deg2rad(jpos));
%             disp("robot");
%             disp(robot_pos(1:3,4));

            % compute the human's intention
            [h_goal, h_goal_idx, goal_probs] = human_intent(human_goals, human_pos_buf);
%             disp("human goal: " + pieces(h_goal_idx));

            % compute the robot's best response goal
            r_goal_name = select_robot_goal(goal_locs, h_pieces_active, r_pieces_active,...
                ROB_GOAL_MODE, human_pos_buf);
%             disp("robot goal: " + r_goal_name);
            
            % switch state to 'ACT' if they've moved and we have the same
            % prediction twice in a row
            human_vel = human_pos_buf(:,end) - human_pos_buf(:,end-1);
            disp(norm(human_vel) + " " + pieces(h_goal_idx));
            % TODO: also check that the person has been moving for this
            % time
            is_consistent = (prev_prev_h_pred == prev_h_pred) && ...
                (prev_h_pred == h_goal_idx);
            if (norm(human_vel) > 0.03) && is_consistent &&...
                    loop_counter > 20
               STATE = 'ACT'; 
            end
            prev_prev_h_pred = prev_h_pred;
            prev_h_pred = h_goal_idx;
            
        case 'ACT'
            % TODO; send pose command to the robot (only once), otherwise continue
            % waiting until pause_time has elapsed
            % move the robot to its selected goal
            if ~rob_cmd_sent
                disp('ACT');
                r_goal = goal_locs.(r_goal_name).pick.up;
                sequence_number = sequence_number + 1;
                pause_time = 2.05;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                r_goal = goal_locs.(r_goal_name).pick.down;
                sequence_number = sequence_number + 1;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                r_goal = goal_locs.(r_goal_name).pick.rotate;
                sequence_number = sequence_number + 1;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                r_goal = goal_locs.neutral;
                sequence_number = sequence_number + 1;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                r_goal = goal_locs.(r_goal_name).drop.up;
                sequence_number = sequence_number + 1;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                r_goal = goal_locs.(r_goal_name).drop.down;
                sequence_number = sequence_number + 1;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                r_goal = goal_locs.(r_goal_name).drop.detach;
                sequence_number = sequence_number + 1;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                r_goal = goal_locs.neutral;
                sequence_number = sequence_number + 1;
                move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);
                
                t0 = tic;
                rob_cmd_sent = true;
            else
                disp('cmd sent');
                % wait until pause_time has elapsed
%                 t1 = toc(t0);
%                 if t1 >= pause_time
%                     disp('finished moving');
%                     break
%                 end
            end
    end
    % TODO: if the human is close enough to any particular goal and not moving,
    % say that is their goal
    
%     t_elapsed = toc(curr_time);
%     disp(t_elapsed);
%     if t_elapsed > 8
%         break
%     end
%     disp(loop_counter);
    loop_counter = loop_counter + 1;
end
disp('finished cleanly');
%%
% comm.endSTMO;
% pause(0.5);
% %%
% stop(tg);
% if MODE == 'CamerINTR'
%     stop(depthVid);
% end

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