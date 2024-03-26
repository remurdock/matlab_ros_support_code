%Controlling Gripper
%Instatiate Gripper Actions and Messages
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory')
gripGoal    = rosmessage(grip_client);
gripPos     = 0.22;
gripGoal    = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal)