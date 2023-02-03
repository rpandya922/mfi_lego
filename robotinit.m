ROBOT = 'LRMate200iD7L';
MODE = 'CamerINTR';%'CamerINTR'; %'CamerINTR'; % HumanINTR for mouse control; HuRobINTR: perception using camera.W
enbSSA = 0; % 1: enable SSA. 0: disable SSA.

load_dependencies;
robot = robot_property(ROBOT);

% Setup Target PC
tg = slrealtime('fanuc_speedgoat');
stop(tg);
load(tg, 'icl_control');
start(tg);
comm=SLRTComm;
comm.timeout=20;
comm.startSTMO;
pause(0.5);
comm.setRobotProperty(robot.nlink,robot.DH,robot.cap,robot.base,robot.ssa_margin);
[jpos, jvel, SSA_status, controller_status] = comm.getRobData;
sequence_number = 0;

switch MODE
    case 'HumanINTR'
        init_pos = [1,1];
        % Intialize figure
        fighandle = initialize_figure_interact(2, [-2, 3], [-2, 2], [0, 3], [1, -1, 2], 1, 1);
        % Calibration
        text1handle = text(0, max(ylim)+1, max(zlim)+0.5, 'Please calibrate...');
        [Center, URCorner] = calibration(init_pos);
        set(fighandle(1), 'currentaxes', fighandle(2))
        set(text1handle, 'string','Test runing...')
    case 'CamerINTR'
        init_pos = [0,0.5];
        %%%%%%%%%%%%%%%%
        % Start Kinect
        %%%%%%%%%%%%%%%%
        depthVid = videoinput('kinect',2);
        prop = getselectedsource(depthVid);
        prop.EnableBodyTracking = 'on';
        framesPerTrig = 6;
        set(depthVid,'TriggerRepeat',Inf);%TriggerRepeat
        set(depthVid,'FramesPerTrigger',framesPerTrig);%FramesPerTrigger
        set(depthVid,'FrameGrabInterval',1);%FrameGrabInterval
        set(depthVid,'Timeout',70);
        start(depthVid)
        Trans = load("Calibration/camera_transformation.txt");
        fighandle = initialize_figure_interact(2, [-2, 3], [-2, 2], [0, 3], [1, -2, 1], 1, 0);
        text1handle = text(0, max(ylim)+1, max(zlim)+0.5, 'Please calibrate...');
        set(text1handle, 'string','Test runing...')
end