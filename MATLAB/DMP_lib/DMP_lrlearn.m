function [DMP]=DMP_lrlearn(y, DMP)


% Locally weighetd regression DMP,  


% input measured values 
%   y matrix of signals
%   y,  ... position(t)
%   dy, ... velocity(t)
%   ddy, ... acceleration(t)
%   dt, ... sample time
%   DMP, ... DMP parameters 

% DMP parameters
%   N,  ... number of Gaussian kernel functions
%   w,  ... weight vector of size(Nx1)
%   c,
%   sigma2
%   tau
%   a_x
%   a_z


%% params - global
[NT,NS] = size(y);
DMP.y0 = y(1,:);    % initial value
DMP.goal  = y(NT,:); % goal - final value
DMP.tau = (NT-1)*DMP.dt;
%% generate derivatives
dy=diff(y)/DMP.dt;
dy=[zeros(1,NS);dy];
ddy=diff(dy)/DMP.dt;
ddy=[zeros(1,NS);ddy];
DMP.dy0 = dy(1,:);    % initial value

%% init params for target traj. and fitting
x = 1;
dx = 0;
P = ones(DMP.N,NS)*1000;    % initial large covariance
DMP.w = zeros(DMP.N,NS);   % initial weights

%%% gausian kernel functions
c_lin=linspace(0,1,DMP.N);
DMP.c=exp(-DMP.a_x * c_lin);
DMP.sigma2=(diff(DMP.c)*0.75).^2;
%DMP.sigma2=(diff(DMP.c)*DMP.q).^2;
DMP.sigma2=[DMP.sigma2,DMP.sigma2(end)];


cutoff = 0.001;
lambda = 0.995;

%% fit all points of the trajectory
for t=1:NT,
    %% the weighted sum of the locally weighted regression models
    psi=exp(-0.5*(x-DMP.c).^2./DMP.sigma2)';
    %% derivatives
    dx=-DMP.a_x*x;
    %% temporal scaling
    dx = dx/DMP.tau;
    %% Euler integration
    x=x+dx*DMP.dt;

    ind = psi > cutoff;
    
    for k = 1:NS,
        % target for fitting - expected fx for perfect fitting
        ft = (DMP.tau^2*ddy(t,k) - DMP.a_z*(DMP.a_z/4*(DMP.goal-y(t,k))- DMP.tau*dy(t,k)));
        %% recursive regression step
    
        P(ind,k) = (P(ind,k)-(P(ind,k).^2*x^2)./(lambda./psi(ind)+x^2*P(ind)))/lambda;
        e = ft-x*DMP.w(ind,k);
        DMP.w(ind,k) = DMP.w(ind,k) + psi(ind).*P(ind)*x.*e;
    end;
end   


