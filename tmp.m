% load('goal_locs.mat');
% pieces = ["red_1x2" "red_2x6" "red_1x8" "orange_1x2"];
% human_goals = [];
% for i=1:length(pieces)
%     human_goals = [human_goals goal_locs.(pieces(i)).pick.xyz_h];
% end
% 
% robot_goals = [];
% for i=1:length(pieces)
%     robot_goals = [robot_goals goal_locs.(pieces(i)).pick.xyz];
% end

% transforms
human = [1.0265 -0.1572 1.0347;
%          1.0262 -0.1575 1.0351;
         0.7829 0.0282 1.2483;
%          0.7561 0.0538 1.1973;
         0.8815 0.1564 1.0799;
%          0.8900 0.1594 1.0885;
%          0.8291 0.0179 1.1409;
         0.8293 0.0175 1.1407];
human_goals = human';
robot = [0.0355 -0.1390 1.6032;
%          0.0355 -0.1390 1.6032;
         0.7891 0.4849 0.6470;
%          0.7891 0.4849 0.6470;
         -0.4191 -0.1396 0.8221;
%          -0.4191 -0.1396 0.8221;
%          0.3131 0.0162 1.1496;
         0.3131 0.0162 1.1496];
robot_goals = robot';
     
% compute transform
[trans, inlier_index, status] = estimateGeometricTransform3D(human, robot, 'rigid');

% construct homogeneous tranform matrix
T = eye(4);
T(1:3,1:3) = trans.Rotation;
T(1:3,4) = trans.Translation;

idx = 3;
g1 = [human_goals(:,idx); 1];
g1_t = T*g1;
disp("h->r");
disp(g1_t);
disp(robot_goals(:,idx));

% g2 = [robot_goals(:,1); 1];
% g2_t = T*g2;
% disp("r->h");
% disp(g2_t);
% disp(human_goals(:,1));
