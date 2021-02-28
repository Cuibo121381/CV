function send2Arm(q,robot)
% send joint vector to the simulation

Arm_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
Arm_send.TimeFromStart.Nsec = 100;

Arm_send.Positions = q;
robot.ArmM.Points= Arm_send;
send(robot.ArmP,robot.ArmM);

% data = get_Panda_data(robot);
% data.Arm.Actual.Positions
% q
% 
% while abs(data.Arm.Actual.Positions - q') < 0.05
%     data = get_Panda_data(robot);
%     pause(0.1)
% end

end