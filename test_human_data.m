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
% ROB_GOAL_MODE = 'naive';

% move home
% home_pos = [0.445; 27.224; -44.181+27.224; 2.789; -49.112; -4.930];
% pause_time = 4;
% move_to_goal(home_pos, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, true);

pose_sent = false;
pause_time = 2.2;
curr_time = tic;

human_pos_buf = zeros(3, 10);

% load('goal_locs_ravi.mat');
load('goal_locs.mat');
% pieces = ["red_1x2" "red_2x6" "red_1x8" "orange_1x2"];

h_pieces = ["green_1x4" "blue_1x4" "pink_1x4" "red_1x8" "red_2x6"...
    "yellow_1x6_b" "yellow_1x6_t"];
human_goals = [];
for i=1:length(h_pieces)
    human_goals = [human_goals goal_locs.(h_pieces(i)).pick.xyz_h];
end
h_goal_curr = [];
h_goal_picked = false;

h_pieces_active = logical([1 0 0 1 1 0 1]);
r_pieces = ["red_1x2" "orange_1x2" "red_2x6" "red_1x8"];
r_piece_to_idx = containers.Map({'red_1x2' 'orange_1x2' 'red_2x6' 'red_1x8'},...
    {1 2 3 4});
r_pieces_active = logical([1 1 1 1]);

STATE = 'SENSE'; % 'MOVE'
prev_h_pred = -1;
prev_prev_h_pred = -1;
rob_cmd_sent = false;
h_vel_thresh = 0.03;

% for robot movements in loop
pause_time = 2.05;
acting_idx = 1;
robot_done = false;

% for data saving
h_xyz_wrist_data = [];
r_xyz_ee_data = []; % end effector xyz positions
r_joint_data = []; % joint positions
times = [];
loop_counter = 0;
init_time = tic;
h_picked_time = tic;
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
    % save the human's current wrist position to buffer
    human_pos_buf = circshift(human_pos_buf, [0, -1]);
    human_pos_buf(:,end) = wrist_pos;
    
    % check if human has been close to an active goal for a while
    h_vel = human_pos_buf(:,end) - human_pos_buf(:,end-1);
    h_vel_long = human_pos_buf(:,end) - human_pos_buf(:,end-3);
    % rate-limit logic (make sure human moves before checking again)
%     disp(norm(h_vel));
    if norm(h_vel) >= h_vel_thresh
%     if norm(h_vel_long) >= 0.0
        h_goal_picked = false;
    end
%     h_curr_time = toc(h_picked_time);
%     if h_curr_time > 1
%         h_goal_picked = false;
%     end
    if norm(h_vel) < h_vel_thresh && ~h_goal_picked % human is not currently moving
        % check distances to goals
        dists = vecnorm(human_goals - human_pos_buf(:,end), 2, 1);
        dists(~h_pieces_active) = inf;
%         disp(dists);
%         disp("human");
%         disp(human_pos_buf(:,end));
%         disp("goal");
%         disp(goal_locs.green_1x4.pick.xyz_h);
        % TODO: make this check if human has been near goal for > 0.5 secs
        if min(dists) < 0.05 % human is by this goal
%         if true % temporary
            [~, h_goal_idx_curr] = min(dists);
            % set this to be the human's "current" goal 
            h_goal_curr = h_pieces(h_goal_idx_curr);
            disp("human picked goal " + h_goal_curr);
            % compute which goals should become (in)active
            switch h_goal_curr
                case "green_1x4"
                    h_pieces_active(1) = 0; % green inactive
                    h_pieces_active(3) = 1; % pink active
                case "blue_1x4"
                    h_pieces_active(2) = 0; % blue inactive
                case "pink_1x4"
                    h_pieces_active(3) = 0; % pink inactive
                    h_pieces_active(2) = 1; % blue active
                case "red_1x8"
                    h_pieces_active(4) = 0; % red 1x8 inactive
                    r_pieces_active(4) = 0;
                case "red_2x6"
                    h_pieces_active(5) = 0; % red 2x6 inactive
                    r_pieces_active(3) = 0;
                case "yellow_1x6_b"
                    h_pieces_active(6) = 0; % yellow 1x6 bottom inactive
                case "yellow_1x6_t"
                    h_pieces_active(7) = 0; % yellow 1x6 top inactive
                    h_pieces_active(6) = 1; % yellow 1x6 bottom active
            end
            h_goal_picked = true;
            h_picked_time = tic;
        end
    end
%     disp(h_pieces_active);
%     disp(r_pieces_active);
    switch STATE
        case 'SENSE'
            [jpos, jvel, SSA_status, controller_status] = comm.getRobData;
            robot_pos = FK(robot, deg2rad(jpos));

            % compute the human's intention
            [h_goal, h_goal_idx, goal_probs] = human_intent(human_goals, human_pos_buf, h_pieces_active);

            % compute the robot's best response goal
            if h_goal_picked
                h_goal = h_goal_curr;
                h_goal_idx = h_goal_idx_curr;
                r_goal_name = select_robot_goal(goal_locs, h_pieces_active, r_pieces_active,...
                    ROB_GOAL_MODE, h_goal_idx, true);
                switch_state = true;
            else
%                 disp("human goal: " + h_pieces(h_goal_idx));
                r_goal_name = select_robot_goal(goal_locs, h_pieces_active, r_pieces_active,...
                    ROB_GOAL_MODE, human_pos_buf, false);
    %             disp("robot goal: " + r_goal_name);
                % switch state to 'ACT' if they've moved and we have the same
                % prediction twice in a row
                human_vel = human_pos_buf(:,end) - human_pos_buf(:,end-1);
    %             disp(norm(human_vel) + " " + h_pieces(h_goal_idx));
                % TODO: also check that the person has been moving for this
                % time
%                 is_consistent = (prev_prev_h_pred == prev_h_pred) && ...
%                     (prev_h_pred == h_goal_idx);
                is_consistent = (prev_prev_h_pred == prev_h_pred);
                if loop_counter == 20
                    disp('waiting for human action');
                end
                if ROB_GOAL_MODE == "naive"
%                    switch_state = (loop_counter > 20);
                    switch_state = (norm(human_vel) > h_vel_thresh) && is_consistent &&...
                    loop_counter > 20;
                else
                    switch_state = (norm(human_vel) > h_vel_thresh) && is_consistent &&...
                    loop_counter > 20;
                end
            end
            
            if switch_state
                STATE = 'ACT';
                disp("robot goal: " + r_goal_name);
                % get the goal positions for the robot all in an array
                robot_goals = zeros(6, 8);
                robot_goals(:,1) = goal_locs.(r_goal_name).pick.up;
                robot_goals(:,2) = goal_locs.(r_goal_name).pick.down;
                robot_goals(:,3) = goal_locs.(r_goal_name).pick.rotate;
                robot_goals(:,4) = goal_locs.(r_goal_name).pick.up;
                robot_goals(:,5) = goal_locs.(r_goal_name).drop.up;
                robot_goals(:,6) = goal_locs.(r_goal_name).drop.down;
                robot_goals(:,7) = goal_locs.(r_goal_name).drop.detach;
                robot_goals(:,8) = goal_locs.neutral;
                1;
            end
            prev_prev_h_pred = prev_h_pred;
            prev_h_pred = h_goal_idx;
            
        case 'ACT'
            % TODO; send pose command to the robot (only once), otherwise continue
            % waiting until pause_time has elapsed
            % move the robot to its selected goal
            if ~rob_cmd_sent
                if ~robot_done
                    disp('ACT');
                    % set this goal for the robot to be inactive
                    g_idx = r_piece_to_idx(r_goal_name);
                    r_pieces_active(g_idx) = false;
                    if r_goal_name == "red_1x8"
                        h_pieces_active(4) = 0;
                    elseif r_goal_name == "red_2x6"
                        h_pieces_active(5) = 0;
                    end

                    % get the current goal to be sent
                    r_goal = robot_goals(:,acting_idx);
                    sequence_number = sequence_number + 1;
                    move_to_goal(r_goal, enbSSA, comm, sequence_number, traj_hz, resample_hz, ovr, pause_time, false);

                    t0 = tic;
                    rob_cmd_sent = true;
                    acting_idx = acting_idx + 1;
                    if acting_idx > 8
                        % assign the next goal to the robot
                        acting_idx = 1;
                        if all(~r_pieces_active)
                            % robot is done picking up its last piece
                            robot_done = true;
                        end
                        STATE = 'SENSE';
                    end
                    disp('cmd sent');
                else
                    disp("robot done");
                end
            else
                t1 = toc(t0);
                if t1 >= pause_time
                    rob_cmd_sent = false;
                end
            end
    end

    loop_counter = loop_counter + 1;
    % saving data
    times = [times toc(init_time)];
    h_xyz_wrist_data= [h_xyz_wrist_data wrist_pos];
    [jpos, ~, ~, ~] = comm.getRobData;
    r_xyz = FK(robot, deg2rad(jpos));
    r_xyz_ee_data = [r_xyz_ee_data r_xyz(1:3,4)];
    r_joint_data = [r_joint_data jpos];
    loop_counter = loop_counter + 1;
    
    if all(~r_pieces_active) && all(~h_pieces_active) && robot_done
        break; % finished moving all pieces
    end
end
disp("time taken: " + (times(end) - times(1)));
disp('finished cleanly');
%% save trial data
user = "ravi";
test_idx = 7;
filename = user + "_" + ROB_GOAL_MODE + "_test" + test_idx + ".mat";
save(filename, 'times', 'h_xyz_wrist_data', 'r_xyz_ee_data', 'r_joint_data');
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
% pause(pause_time);
end