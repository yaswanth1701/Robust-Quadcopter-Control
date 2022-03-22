function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
kp1=50;
kd1=10;
kp2=100;
kd2=10;
kp3=50;
kd3=0.05;
r1des_dd=des_state.acc(1);
r2des_dd=des_state.acc(2);
r3des_dd=des_state.acc(3);
r1des_d=des_state.vel(1);
r2des_d=des_state.vel(2);
r3des_d=des_state.vel(3);
r1des=des_state.pos(1);
r2des=des_state.pos(2);
r3des=des_state.pos(3);
r1_d=state.vel(1);
r2_d=state.vel(2);
r3_d=state.vel(3);
r1=state.pos(1);
r2=state.pos(2);
r3=state.pos(3);
r1c_dd=r1des_dd+kd1*(r1des_d-r1_d)+kp1*(r1des-r1);
r2c_dd=r2des_dd+kd2*(r2des_d-r2_d)+kp2*(r2des-r2);
r3c_dd=r3des_dd+kd3*(r3des_d-r2_d)+kp3*(r3des-r3);

% Thrust

F = params.mass*(params.gravity+r3c_dd);

% Moment
kdp=50;
kpp=1500;
kdt=50;
kpt=1500;
kdps=50;
kpps=500;
phi_c=(r1c_dd*sin(des_state.yaw)-r2c_dd*cos(des_state.yaw))/(params.gravity);
theta_c=(r1c_dd*cos(des_state.yaw)+r2c_dd*sin(des_state.yaw))/(params.gravity);
ps_c=des_state.yaw;

M =params.I*[kpp*(phi_c- state.rot(1))-kdp*(state.omega(1));kpt*(theta_c-state.rot(2))-kdt*(state.omega(2));kpps*(ps_c-state.rot(3))+kdps*(des_state.yawdot- state.omega(3))];
% =================== Your code ends here ===================

end
