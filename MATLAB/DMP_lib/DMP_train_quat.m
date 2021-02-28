function DMP = DMP_train_quat(y, time, DMP)
% Encodes the input trajectory y sampled at time with a discrete DMP.
% First, the derivatives of the provided trajectory are calculated. Then,
% the DMP structure is defined and the parameters of the basis functions 
% are calculated. The weights w are then calculated from the input trajectory
% in the least square sense.
%
% INPUTS:
%   y: the input trajectory. Please make sure that:
%   y(:,1:3) -> rapresents the position in cartesian space
%   y(:,4:7) -> rapresents the orientation expressed with quaternions
%   time: times at which the trajectory is sampled
%   DMP.N: number of basis functions to encode the trajectory. Optional, default = 100.
%   DMP.a_z: constant alpha of the dynamical system. Optional, default = 48.
%   DMP.a_x: constant alpha of the canonical system. Optional, default = 2.
% 
% OUTPUT:
%   DMP: the DMP structure containing all the calculated parameters.
% 
%   The DMP has 6 sets of weights, where the first three rapresent the
%   weights for the positional part and the last three rapresent the
%   weights for the orientational part.


%% check parameters, some are optional
if ~exist('DMP')
  DMP.N = 100;
elseif ~isfield(DMP, 'N')
  DMP.N = 100; 	% number of basis functions to encode the path
end
if ~isfield(DMP, 'a_z')
  DMP.a_z = 48;	% DMP stifness
end
if ~isfield(DMP, 'b_z')
  DMP.b_z = DMP.a_z / 4;
end
if ~isfield(DMP, 'a_x')
  DMP.a_x = 2;
end
if ~isfield(DMP, 'is_zero')
  DMP.is_zero = 1;
end

%% define the rest of the DMP parameters

DMP.tau = time(end);    % DMP time constant
DMP.goal  = y(end,:);   % goal - final value
DMP.w = zeros(DMP.N,6);
DMP.dt = time(2) - time(1);

% define Gausian kernel functions
c_lin = linspace(0, 1, DMP.N);
DMP.c = exp(-DMP.a_x * c_lin);	      % centers of gaussians
DMP.sigma2 = (diff(DMP.c)*0.75).^2;	  % widths of gaussians
DMP.sigma2 = [DMP.sigma2, DMP.sigma2(end)];


%% compute postitional derivatives
p_traj = y(:,1:3);
[NT, NS]  = size(p_traj);
if length(time) == 1
  time = linspace(0, (NT-1)*time, NT);
end
time = time - time(1);
for dof = 1:NS
  dp_dt(:,dof) = gradient(y(:,dof), time);
  ddp_dt(:,dof) = gradient(dp_dt(:,dof), time);
end

%% Compute orientation derivatives
q_start = y(1,4:7)';
q_goal = y(end,4:7)';
q_traj = y(:,4:7);

% Generate quaternion data, angular velocities and angular accelerations
N = length(time);
q = zeros(4,N);
qq = zeros(4,N);
dqq = zeros(4,N);
omega = zeros(3,N);
domega = zeros(3,N);


% Normalize quaternions 
for i = 1:N
    tmp = norm([q_traj(i,1); q_traj(i,2:4)']); 
    q(1,i) = q_traj(i,1) / tmp;
    q(2:4,i) = q_traj(i,2:4)' / tmp;
    qq(:,i) = q(:,i);
end

% Calculate derivatives
for j = 1:4
    dqq(j,:) = gradient(qq(j,:), time);
end

% Calculate omega (angular speed) and domega (agular acceleration)
for i = 1:N
    dq(1,1) = dqq(1,i);
    for j = 1:3
        dq(1 + j,1) = dqq(j+1,i);
    end
    omega_q = quat_mult_conj(dq, q(:,i)); 
    omega(:,i) = 2*omega_q(2:4);
end
for j = 1:3
    domega(j,:) = gradient(omega(j,:), time);
end

omega(:,1) = [0; 0; 0];
omega(:,N) = [0; 0; 0];
domega(:,1) = [0; 0; 0];
domega(:,N) = [0; 0; 0];

%%

DMP.y0 = y(1,:);        % initial state
DMP.dy0 = [dp_dt(1,:) omega(:,1)'];   % initial velocity

%% compute weights by linear regression for the positional part
ft_x = zeros(NT, NS);
A_x = zeros(NT, DMP.N);
% progression of phase
x = exp(-DMP.a_x * time / DMP.tau);
% target for fitting
for dof = 1:NS
  ft_x(:,dof) = ddp_dt(:,dof)*DMP.tau^2 - DMP.a_z * (DMP.b_z * (DMP.goal(dof) - p_traj(:,dof)) - dp_dt(:,dof) * DMP.tau);
end

for k = 1:NT
  % values of kernel functions at current phase
  psi = exp(-0.5 * (x(k) - DMP.c).^2 ./ DMP.sigma2)';
  % one row of regression matrix
  A_x(k,:) = x(k) * psi / sum(psi);
end


%% compute weights by linear regression for the quaternion part
diag = 2*quat_diff_log(q_goal,q_start); % The difference between ther start orientation and the end orientation
d = 1 ./ diag';
fx_q = zeros(N,3);
A_q = zeros(N, DMP.N);
for i = 1:N
    fx_q(i,:) = d.*(DMP.tau^2*domega(:,i) + DMP.a_z*DMP.tau*omega(:,i) - DMP.a_z*DMP.b_z*2*quat_diff_log(q_goal,q(:,i)))';
    
    x = exp(-DMP.a_x*time(i)/DMP.tau);
    psi = exp(-(x-DMP.c).^2./(2*DMP.sigma2));
    A_q(i,:) = x*psi/sum(psi);
end

%%
DMP.w(:,1:3) = A_x\ft_x;
DMP.w(:,4:6) = (A_q \ fx_q);

end

function q_out = quat_log_vec(q_in)

    % Calculates logarithm of a quaternion
    % Output is a 3x1 vector, because for the DMP we use only the vector part


    q_scalar = q_in(1);
    q_vector = q_in(2:4);

    if norm(q_vector) > 1.0e-12 && q_scalar < 0.999999
        q_out = (acos( q_scalar ) * q_vector) / norm(q_vector);
    else
        q_out = [0; 0; 0];
    end

end

function q = quat_mult_conj(q1, q2)

    % Calculates logarithm of orientation difference between quaternions.
    % Output is vector

    q1_scalar = q1(1);
    q1_vector = q1(2:4);
    q2_scalar = q2(1);
    q2_vector = q2(2:4);

    % Conjugate the q2 quaternion 
    q2c_scalar = q2_scalar;
    q2c_vector = -q2_vector;


    q_scalar = q1_scalar * q2c_scalar - q1_vector' * q2c_vector;
    q_vector = q1_scalar * q2c_vector + q2c_scalar * q1_vector + cross(q1_vector, q2c_vector);

    q = [q_scalar; q_vector];

end

function q_out = quat_diff_log(q1, q2)
    
    q_out = quat_log_vec(quat_mult_conj(q1,q2));
    
end