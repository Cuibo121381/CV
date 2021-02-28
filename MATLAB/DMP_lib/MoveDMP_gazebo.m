%%
clc
clear all
close all
dt=1/30;
path('../ROS_lib',path)
load test_trj.mat

%% TODO: Implement the encoding step for the Joint DMP from the previous
%exercise.


%% Execute in simulation
rosinit
 try
    %connect to sim robot
    
    [robot,q0] = init_Panda(dt);
    Startjoints = qJoints(1,:);
    
    %% move to init position
    Jmove_Ros(Startjoints,2,robot);
    disp('start DMP')
    pause(3)
    
    i=1;
    tn=0;
    st = tic;
    
    %setup rosmessage for sending the joint positions to ROS
    Arm_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    Arm_send.TimeFromStart.Nsec = 330000000;
    
    while Sj.x > Xmin
        
        %% TODO: JDMP integration simmilar as in the previous exercise
        %% Joint DMP
        
        
        %% Send to robot

        Arm_send.Positions = Sj.y;
        Arm_send.Velocities = Sj.z;
        robot.ArmM.Points= Arm_send;
        send(robot.ArmP,robot.ArmM);
  
        %% sinhronisation wiht the simulator in order to send the joint positions with a constant rate.
        tn = tn+dt;
        if tn>toc(st)
            pause(tn-toc(st))
        end
        i=i+1;
    end
    
    disp('finished')
    %% move to start position when the DMP is executed 
    Jmove_Ros([0 0 0 -pi/2 0 pi/2 0],2,robot);
    
    pause(3)
    
catch ME
    ErrorTrap(ME);
end

rosshutdown
