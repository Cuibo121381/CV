function [data] = get_Panda_data(robot)
%% read arm and gripper joint state
data.Arm = receive(robot.ArmS,3);
data.Hand = receive(robot.HandS,3);
end