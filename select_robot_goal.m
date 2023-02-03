function r_goal_name = select_robot_goal(goal_locs, h_pieces_active, r_pieces_active,...
    ROB_GOAL_MODE, human_pos)
% list of all pieces for human
h_pieces = ["green_1x4" "blue_1x4" "pink_1x4" "red_1x8" "red_2x6"...
    "yellow_1x6_b" "yellow_1x6_t"];
% h_pieces_active = logical([1 0 0 1 1 0 1]);
n_goals = length(h_pieces);
h_pick_goals = zeros(3, n_goals);
h_drop_goals = zeros(3, n_goals);
for i=1:n_goals
    h_pick_goals(:,i) = goal_locs.(h_pieces(i)).pick.xyz_h;
    h_drop_goals(:,i) = goal_locs.(h_pieces(i)).drop.xyz_h;
end

r_pieces = ["red_1x2" "orange_1x2" "red_2x6" "red_1x8"];
% r_pieces_active = logical([1 1 1 1]);
n_goals_r = length(r_pieces);

% create mapping from each human piece to a robot piece
h_to_r_piece = containers.Map({'green_1x4' 'blue_1x4' 'pink_1x4' 'red_1x8'...
     'red_2x6' 'yellow_1x6_b' 'yellow_1x6_t'}, {'red_1x2' 'red_1x2' 'red_1x2'...
     'red_1x8' 'red_2x6' 'orange_1x2' 'orange_1x2'});
h_piece_idx = containers.Map({'green_1x4' 'blue_1x4' 'pink_1x4' 'red_1x8'...
     'red_2x6' 'yellow_1x6_b' 'yellow_1x6_t'}, {1 2 3 4 5 6 7});
 
% mapping from each robot piece to set of human pieces
r_to_h_pieces = containers.Map({'red_1x2' 'orange_1x2' 'red_2x6' 'red_1x8'}, ...
    {["green_1x4" "blue_1x4" "pink_1x4"], ["yellow_1x6_b" "yellow_1x6_t"],...
    ["red_2x6"], ["red_1x8"]});

h_pieces_active_from_r = ones(1, n_goals);
for i=1:n_goals_r
    r_piece = r_pieces(i);
    h_pieces_mapped = r_to_h_pieces(r_piece);
    if ~r_pieces_active(i)
        % this robot piece has already been moved
        for j=1:length(h_pieces_mapped)
            h_pieces_active_from_r(h_piece_idx(h_pieces_mapped(j))) = 0;
        end
    end
end

% locations of robot pick and drop positions
r_pick_goals = zeros(3, n_goals);
r_drop_goals = zeros(3, n_goals);
for i=1:n_goals
    h_piece_name = h_pieces(i);
    r_piece_name = h_to_r_piece(h_piece_name);
    r_pick_goals(:,i) = goal_locs.(r_piece_name).pick.xyz;
    r_drop_goals(:,i) = goal_locs.(r_piece_name).drop.xyz;
end

[h_goal, h_goal_idx, goal_probs] = human_intent(h_pick_goals, human_pos);

switch(ROB_GOAL_MODE)
    case 'proactive'
        % picks the goal that will influence the human the least based on
        % score
        % create scores for all human goals
        goal_scores = score_goals(h_pick_goals, h_drop_goals, r_pick_goals,...
            r_drop_goals);

        % using intention prediction, select goal for robot that doesn't interfere
        % get the scores relating to this goal
        h_goal_scores = goal_scores(h_goal_idx,:)';
        % use set any goal with h_pieces_active(i) == 0 to have negative
        % score
        h_goal_scores(~h_pieces_active) = -inf;
        
        % additionally filter based on which goals are active for robot
        h_goal_scores(~h_pieces_active_from_r) = -inf;
        
        % compute probability that the robot should go to each goal
        robot_goal_probs = softmax(h_goal_scores);
        [~, robot_goal_idx] = max(robot_goal_probs);
        
        % get the name of the corresponding robot goal
        r_goal_name = h_to_r_piece(h_pieces(robot_goal_idx));
        
    case 'naive'
        % picks the goal that's closest to the robot (by xyz position)
        disp("NOT IMPLEMENTED");
    case 'responsive'
        % picks the closest goal that the human isn't going towards
        disp("NOT IMPLEMENTED");
end

end

% function scores = score_goals(goal_locs, pieces)
function scores = score_goals(h_pick, h_drop, r_pick, r_drop)
shape = size(h_pick);
n_goals = shape(2);

score_mat = zeros(n_goals, n_goals);

for i=1:n_goals
    for j=1:n_goals
        % human picks goal i, robot picks goal j
        pickup_dist = norm(h_pick(:,i) - r_pick(:,j));
        dropoff_dist = norm(h_drop(:,i) - r_drop(:,j));
        score_mat(i,j) = -1/pickup_dist - 1/dropoff_dist;
    end
end

scores = score_mat;
end