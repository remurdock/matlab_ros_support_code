%Prepare script to recieve move_to_can code
clear
clc

%Set up link between ROS and Gazebo
rosshutdown
rosinit("192.168.230.128")

%Prepare World to recieve 
goHome('qz')
resetWorld

%Instatiate Actions and Messages
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                          'control_msgs/FollowJointTrajectory') 
trajGoal = rosmessage(trajAct)
trajAct.FeedbackFcn = []; 
jointSub = rossubscriber("/joint_states")
jointStateMsg = jointSub.LatestMessage

%Set up Inverse Kinematics
UR5e = loadrobot('universalUR5e', DataFormat="row") %Load Robot

%Adjust the forward kinematics to match the URDF model in Gazebo
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver

ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation); % receive current robot configuration

jointStateMsg = receive(jointSub,3) % receive current robot configuration

initialIKGuess = homeConfiguration(UR5e)

%Copy the correct order from jointStateMsg.Position to our structure initialIKGuess
jointStateMsg.Name
% update configuration in initial guess
initialIKGuess(1) = jointStateMsg.Position(4);  % Shoulder Pan
initialIKGuess(2) = jointStateMsg.Position(3);  % Shoulder Tilt
initialIKGuess(3) = jointStateMsg.Position(1);  % Elbow
initialIKGuess(4) = jointStateMsg.Position(5);  % W1
initialIKGuess(5) = jointStateMsg.Position(6);  % W2
initialIKGuess(6) = jointStateMsg.Position(7);  % W3
show(UR5e,initialIKGuess)

%Set End Effector Pose
gripperX = -0.04;
gripperY = 0.8;
gripperZ = 0.115;

gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians

tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess) %Compute IK

%Send Joint Trajectory Goal
UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

trajGoal = packTrajGoal(UR5econfig,trajGoal)
sendGoal(trajAct,trajGoal)