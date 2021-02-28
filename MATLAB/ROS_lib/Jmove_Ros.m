%==========================================================================
function [Data] = Jmove_Ros(q,t,robot)
%==========================================================================
% join move from teh current joints to the (q) pose in (t) secconds
tn=0.0;
disp('jmove..')

%read robot data
data = get_Panda_data(robot);
q0 = data.Arm.Actual.Positions';
% from last position
if norm(q-q0) < 0.05
    disp('already there')
    return
end

time =  robot.dt:robot.dt:t;
qi = jtraj(q0,q,time);
qdi = diff(qi)/robot.dt;
qdi = [qdi;qdi(end,:)];

Arm_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
Arm_send.TimeFromStart.Nsec = 10000000;

%execute
st = tic;
for i = 1:size(qi,1)
   
    Arm_send.Positions = qi(i,:);
    Arm_send.Velocities = qdi(i,:);
    robot.ArmM.Points= Arm_send;
    send(robot.ArmP,robot.ArmM);
    
    tn = tn+robot.dt;
    if tn>toc(st)
        pause(tn-toc(st))
    end
end

disp('done.');
end