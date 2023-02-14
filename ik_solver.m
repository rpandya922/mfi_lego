function solver = ik_solver(robot)
% ROBOT = 'LRMate200iD7L';
% load_dependencies;
% robot = robot_property(ROBOT);

DH = [robot.DH(:,3) robot.DH(:,4) robot.DH(:,2) robot.DH(:,1)];

robot_tree = rigidBodyTree;
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1, DH(1,:),'dh');
body1.Joint = jnt1;
addBody(robot_tree, body1, 'base');

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4', 'revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5', 'revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6', 'revolute');

setFixedTransform(jnt2, DH(2,:),'dh');
setFixedTransform(jnt3, DH(3,:),'dh');
setFixedTransform(jnt4, DH(4,:),'dh');
setFixedTransform(jnt5, DH(5,:),'dh');
setFixedTransform(jnt6, DH(6,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot_tree, body2, 'body1');
addBody(robot_tree, body3, 'body2');
addBody(robot_tree, body4, 'body3');
addBody(robot_tree, body5, 'body4');
addBody(robot_tree, body6, 'body5');

robot_tree.DataFormat = 'column';

% get the position and orientation of known joint position
% load('goal_locs.mat');
% pose = FK(robot, deg2rad(goal_locs.red_1x2.pick.down));
% pose(1:3,4) = pose(1:3,4) - robot.base;

% try creating IK solver
solver = inverseKinematics('RigidBodyTree', robot_tree);
% weights = [0.25 0.25 0.25 1 1 1];
% init_guess = [0; 0; 0; 0; 0; 0];
% [configSol, solInfo] = solver('body6', pose, weights, init_guess);
% 
% show(robot_tree, configSol) 
% need to add pi/2 to configSol(2) so it's the same as our initial config
end