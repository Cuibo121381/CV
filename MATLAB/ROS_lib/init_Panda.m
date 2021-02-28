function [robot,q0] = init_Panda(dt)
% initialization of subscribers and recievers fot communication with the
% gazebo simulation
disp('--------Initializing Gazebo Subscribers and Publishers---------')
robot.dt=dt;
robot.ArmS = rossubscriber("/panda_arm_controller/state",'BufferSize', 5);
robot.HandS = rossubscriber("/panda_hand_controller/state");
robot.ArmP = rospublisher("/panda_arm_controller/command");
robot.HandP = rospublisher("/panda_hand_controller/command");

robot.ArmData = receive(robot.ArmS,3);
robot.ArmM= rosmessage(robot.ArmP);
robot.HandData = receive(robot.HandS,3);
robot.HandM = rosmessage(robot.HandP);
robot.HandM.JointNames = robot.HandData.JointNames;
robot.ArmM.JointNames = robot.ArmData.JointNames;

q0=robot.ArmData.Actual.Positions;
disp('robot redy to go!!')

end