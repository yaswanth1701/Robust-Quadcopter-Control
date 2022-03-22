function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
e_x= s_des(1) - s(1);
e_vel= s_des(2) - s(2);
z_ddes=0;
Kp = 350;
Kd = 40;
u = params.mass*(params.gravity + Kp*e_x + Kd*e_vel+ (z_ddes));


% FILL IN YOUR CODE HERE


end

