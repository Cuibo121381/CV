function send2Hand(state, robot)
%% move fingers to desied position,
% position is normaized foe each finger between 0 and 0.04
if state >= 0.04
    state = 0.04;
elseif state <= 0.00
    state = 0;
end
Hand_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
Hand_send.TimeFromStart.Nsec = 100;

Hand_send.Positions = [state,state];
robot.HandM.Points= Hand_send;
send(robot.HandP,robot.HandM);
end