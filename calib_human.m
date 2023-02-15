% script to calibrate the wrist positions of the human picking up goals

% load the goal locations
load('goal_locs.mat');

disp("go to yellow 1x6");
pause(5);
% record position at yellow_1x6_t
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
goal_locs.yellow_1x6_t.pick.xyz_h = wrist_pos;
goal_locs.yellow_1x6_b.pick.xyz_h = wrist_pos;
goal_locs.orange_1x2.pick.xyz_h = wrist_pos;
disp("done");

% record red 2x6
disp("go to red 2x6");
pause(5);
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
goal_locs.red_2x6.pick.xyz_h = wrist_pos;
disp("done");

% record red 1x8
disp("go to red 1x8");
pause(5);
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
goal_locs.red_1x8.pick.xyz_h = wrist_pos;
disp("done");

% record red 1x2
disp("go to green 1x4");
pause(5);
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
goal_locs.red_1x2.pick.xyz_h = wrist_pos;
goal_locs.green_1x4.pick.xyz_h = wrist_pos;
goal_locs.pink_1x4.pick.xyz_h = wrist_pos;
goal_locs.blue_1x4.pick.xyz_h = wrist_pos;
disp("done");
