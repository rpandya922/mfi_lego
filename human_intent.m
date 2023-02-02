function [h_goal, h_goal_idx, goal_probs] = human_intent(human_goals, human_pos)
% load('goal_locs.mat');
% pieces = ["red_1x2" "red_2x6" "red_1x8" "orange_1x2"];
% human_goals = [];
% for i=1:length(pieces)
%     human_goals = [human_goals goal_locs.(pieces(i)).pick.xyz];
% end
% 
% % dummy numbers
% human_pos = [0.4 0.3;
%              0.0 0.05;
%              1.5 1.4];
% 
% [h_goal, goal_probs] = predict(human_goals, human_pos);

goals_shape = size(human_goals);
n_goals = goals_shape(2);

human_vel = human_pos(:,end) - human_pos(:,end-1);

% compute unit vector from velocity
vel_unit = human_vel / norm(human_vel);

% compute unit vectors for direction from wrist position to each goal
directions =  zeros(3, n_goals);
for goal_i=1:n_goals
    diff = human_goals(:,goal_i) - human_pos(:,end-1);
    directions(:,goal_i) = diff/norm(diff);
end

dists = [];
for goal_i=1:n_goals
%     dists = [dists norm(directions(:,goal_i) - human_pos(:,end))^2];
    % try computing dist from just xy position
    dir = directions(:,goal_i);
    h_pos = vel_unit;
    dist = norm(dir(1:2) - h_pos(1:2))^2;
    dists = [dists dist];
end
dists_inv = zeros(length(dists), 1);
for i=1:length(dists)
    dists_inv(i) = 1 / dists(i);
end

[~, h_goal_idx] = min(dists);

% return the predicted human goals and associated probabilities
goal_probs = softmax(dists_inv);
h_goal = human_goals(:,h_goal_idx);

end
