%%
clc
% clear all
close all
dt=0.01;
load test_trj.mat

%encode into DMP
[DMP,QDMP] = LearnQDMP(Qpath,dt);

% TO DO: for joint trajectories define a function for calculating a Joint
% DMP parameters Inputs: qJoints and dt Output: JDMP parameters. the
% function should be simmilar as LearnQDMP (without the orientation part)


%%Calculate final phase for the DMP integration
T_f = (length(Qpath)+1)*DMP.dt;
Xmin = exp(-DMP.a_x*T_f/DMP.tau);

%init. states for position dmp
Sp.y = DMP.y0;
Sp.z = zeros(1,7);
Sp.x = 1;

%init. states for position dmp
Sj.y = JDMP.y0;
Sj.z = zeros(1,7);
Sj.x = 1;

%init. states for quaternion dmp
Sq.q = QDMP.q0;
Sq.o = zeros(3,1);
Sq.x = 1;



%%
i=1;
while Sp.x > Xmin
    % position DMP
    [Sp]=DMP_integrate(DMP,Sp,0);
    xN(i,:) = Sp.y;
    
    %%  TODO: Joint DMP integrate Configure the a simillar function as for the position part of the DMP
    
   
    %%
    % Quaternion DMP
    [Sq]=qDMP_integrate(QDMP,Sq,0);
    qQ(i,:) = Sq.q; % just for plotting
    
    i=i+1;
end

%% Plot Position part of the trajectory
plot(xN,'r')  % plot DMP trajectory
hold on
plot(Qpath(:,1:3),'--b') % plot example trajectory

%% Plot Orientation part of the trajectory
figure(2)
plot(qQ,'r') % plot QDMP trajectory
hold on
plot(Qpath(:,4:7),'--b') % plot example trajectory

% Plot Joint trajectories
figure(3)
plot(jN,'r') % plot QDMP trajectory
hold on
plot(qJoints,'--b') % plot example trajectory


