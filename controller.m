function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
m = params.mass;
g= params.gravity;
Ixx = params.Ixx;
kpz=80;
kdz=20;
kpp=1000;
kdp=50;
kpy=50;
kdy=20;
 
 
u1 = m*(g+des_state.acc(2)+kdz*(des_state.vel(2)-state.vel(2))+kpz*(des_state.pos(2)-state.pos(2)));
or=(-1/g)*(des_state.acc(1)+kdy*(des_state.vel(1)-state.vel(1))+kpy*(des_state.pos(1)-state.pos(1)));
u2 = Ixx*(-1*kdp*state.omega(1)+kpp*(or-state.rot(1)));

% FILL IN YOUR CODE HERE

end

