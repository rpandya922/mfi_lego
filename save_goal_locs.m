goal_locs = [];
goal_locs.neutral = [0.445; 27.224; -44.181+27.224; 2.789; -49.112; -4.930];

% transparent red piece 1x2
goal_locs.red_1x2.pick.up = [-12.745; 68.813; -44.296+68.813; 1.512; -46.689; 10.292];
goal_locs.red_1x2.pick.down = [-12.776; 70.904; -45.541+70.904; 1.545; -45.444; 10.276];
goal_locs.red_1x2.pick.rotate = [-13.260; 66.652; -50.950+66.652; -3.225; -27.699; 14.672];
xyz = FK(robot, goal_locs.red_1x2.pick.down);
goal_locs.red_1x2.pick.xyz = xyz(1:3,4);
goal_locs.red_1x2.pick.xyz_h = [1.00; -0.11; 1.09];
% dropping off
goal_locs.red_1x2.drop.up = [10.154; 58.939; -58.411+58.939; 3.656; -35.707; -104.607];
goal_locs.red_1x2.drop.down = [10.100; 60.699; -59.754+60.699; 3.787; -34.362; -104.712];
goal_locs.red_1x2.drop.detach = [10.154; 58.939; -58.411+58.939; 3.656; -35.707; -104.607];
xyz = FK(robot, goal_locs.red_1x2.drop.down);
goal_locs.red_1x2.drop.xyz = xyz(1:3,4);

% solid red 1x8
goal_locs.red_1x8.pick.up = [-6.986; 78.521; -32.095+78.521; 3.777; -59.466; 5.988];
goal_locs.red_1x8.pick.down = [-7.083; 80.970; -33.265+80.970; 3.827; -58.292; 5.991];
goal_locs.red_1x8.pick.rotate = [-7.378; 75.194; -39.412+75.194; 2.663; -42.134; 6.764];
xyz = FK(robot, goal_locs.red_1x8.pick.down);
goal_locs.red_1x8.pick.xyz = xyz(1:3,4);
goal_locs.red_1x8.pick.xyz_h = [1.04; -0.05; 1.06];
% dropping off
goal_locs.red_1x8.drop.up = [5.320; 59.500; -57.462+59.500; 0.193; -33.813; 83.595];
goal_locs.red_1x8.drop.down = [5.317; 61.464; -58.792+61.464; 0.200; -32.483; 83.589];
goal_locs.red_1x8.drop.detach = [9.331; 61.808; -57.902+61.808; 24.711; -38.851; 59.651];
xyz = FK(robot, goal_locs.red_1x8.drop.down);
goal_locs.red_1x8.drop.xyz = xyz(1:3,4);
goal_locs.red_1x8.drop.xyz_h = [0.7938; 0.0893; 1.0382];
% dropoff 0.7938 0.0893 1.0382

% solid red 2x6
goal_locs.red_2x6.pick.up = [-4.357; 62.556; -52.464+62.556; 5.023; -39.302; 1.387];
goal_locs.red_2x6.pick.down = [-4.460; 64.891; -54.029+64.891; 5.200; -37.739; 1.259];
goal_locs.red_2x6.pick.rotate = [-4.762; 61.425; -58.760+61.425; 5.405; -20.921; 1.159];
xyz = FK(robot, goal_locs.red_2x6.pick.down);
goal_locs.red_2x6.pick.xyz = xyz(1:3,4);
goal_locs.red_2x6.pick.xyz_h = [0.95; -0.029; 1.04];
% dropping off
goal_locs.red_2x6.drop.up = [13.748; 60.852; -55.093+60.852; 0.386; -36.148; -104.169];
goal_locs.red_2x6.drop.down = [13.741; 63.165; -56.635+63.165; 0.401; -34.606; -104.179];
goal_locs.red_2x6.drop.detach = [13.748; 60.852; -55.093+60.852; 0.386; -36.148; -104.169];
xyz = FK(robot, goal_locs.red_2x6.drop.down);
goal_locs.red_2x6.drop.xyz = xyz(1:3,4);
goal_locs.red_2x6.drop.xyz_h = [0.8382; 0.1800; 1.1104];
% dropping off location 0.8382 0.1800 1.1104

% orange 1x2
goal_locs.orange_1x2.pick.up = [2.888; 67.695; -44.579+67.695; 3.990; -47.541; -4.672];
goal_locs.orange_1x2.pick.down = [2.779; 70.419; -46.356+70.419; 4.114; -45.761; -4.737];
goal_locs.orange_1x2.pick.rotate = [2.884; 66.118; -51.885+66.118; 6.998; -28.119; -7.480];
xyz = FK(robot, goal_locs.orange_1x2.pick.down);
goal_locs.orange_1x2.pick.xyz = xyz(1:3,4);
goal_locs.orange_1x2.pick.xyz_h = [0.99; 0.01; 1.04];
% dropping off
goal_locs.orange_1x2.drop.up = [9.806; 59.236; -57.301+59.236; 2.913; -34.174; -104.195];
goal_locs.orange_1x2.drop.down = [9.760; 61.290; -58.713+61.290; 3.025; -32.762; -104.285];
goal_locs.orange_1x2.drop.detach = [9.806; 59.236; -57.301+59.236; 2.913; -34.174; -104.195];
xyz = FK(robot, goal_locs.orange_1x2.drop.down);
goal_locs.orange_1x2.drop.xyz = xyz(1:3,4);

goal_locs.yellow_1x6_b.pick.xyz_h = goal_locs.orange_1x2.pick.xyz_h;
goal_locs.yellow_1x6_b.drop.xyz_h = [0.9034; 0.1306; 1.0487];

goal_locs.yellow_1x6_t.pick.xyz_h = goal_locs.orange_1x2.pick.xyz_h;
goal_locs.yellow_1x6_t.drop.xyz_h = [0.8094; 0.1222; 1.0355];

goal_locs.green_1x4.pick.xyz_h = goal_locs.red_1x2.pick.xyz_h;
goal_locs.green_1x4.drop.xyz_h = [0.8656; 0.1407; 1.1089];

goal_locs.pink_1x4.pick.xyz_h = goal_locs.red_1x2.pick.xyz_h;
goal_locs.pink_1x4.drop.xyz_h = [0.8602; 0.0568; 1.0905];

goal_locs.blue_1x4.pick.xyz_h = goal_locs.red_1x2.pick.xyz_h;
goal_locs.blue_1x4.drop.xyz_h = [0.9124; 0.0686; 1.0947];
% dropoff yellow 1x6 bottom 0.9034 0.1306 1.0487
% dropoff yellow 1x6 top 0.8094 0.1222 1.0355
% dropoff green 0.8656 0.1407 1.1089
% dropoff pink 0.8602 0.0568 1.0905
% dropoff blue 0.9124 0.0686 1.0947
save('goal_locs.mat', 'goal_locs');