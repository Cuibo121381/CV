function OpenHand(robot)
%% Open hand,
%
state = 0.04;
Hand_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
Hand_send.TimeFromStart.Nsec = 100;

Hand_send.Positions = [state,state];
robot.HandM.Points= Hand_send;
send(robot.HandP,robot.HandM);
end