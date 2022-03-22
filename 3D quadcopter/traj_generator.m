function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 coff
if nargin > 2
    
    
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    coff=getcoff(waypoints);
    
else
    if(t > traj_time(end))
        t = traj_time(end)-0.0001;
    end
    t_index = find(traj_time >= t,1)-1;
    t_index=max(t_index,1);
    scale=(t-traj_time(t_index))/d0(t_index);
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel=zeros(3,1);
        desired_state.acc=zeros(3,1);
        desired_state.yaw=0;
        desired_state.yawdot=0;
    else
        t0=poly(8,0,scale)';
        t1=poly(8,1,scale)';
        t2=poly(8,2,scale)';
        index=[(t_index-1)*8+1:t_index*8];
        desired_state.pos=coff(index,:)'*t0;
        desired_state.vel=coff(index,:)'*t1.*(1/d0(t_index));
        desired_state.acc=coff(index,:)'*t2.*(1/d0(t_index)^2);
        desired_state.yaw=0;
        desired_state.yawdot=0;
    end 
end 


function [T]=poly(n,k,t)
    T=zeros(n,1);
    D=zeros(n,1);
    for i=1:n
        D(i)=i-1;
        T(i)=1;
    end
    for k=1:k
        for i=1:n
            T(i)=T(i)*D(i);
            if D(i)>0
                D(i)=D(i)-1;
            end
        end
    end
    for i=1:n
        T(i)=T(i)*t^D(i);
    end
    T=T';
end 

function [coff,A,b]=getcoff(waypoints)
    n=size(waypoints,2)-1; %numeber of rows in waypoint matrixs -1 gives number of curves 
    A=zeros(8*n,8*n);
    b=zeros(3,8*n);
    row=1;
    %case 1
    for i=1:n
        A(row,(i-1)*2*n+1:2*n*i)=poly(8,0,0);
        row=row+1;
    end
    for i=1:n
        A(row,(i-1)*2*n+1:2*n*i)=poly(8,0,1);
        row=row+1;
    end
    for i=1:3
        A(row,1:8)=poly(8,i,0);
        row=row+1;
    end
    for i=1:3
        A(row,25:32)=poly(8,i,1);
        row=row+1;
    end
   for i=2:n
    for k=1:6
        A(row,((i-2)*8)+1:i*8)= [poly(8,k,1) -poly(8,k,0)];
        row=row+1; 
    end
   end
   
    for i=1:n
        b(:,i)=waypoints(:,i);
    end
    for i=1:n
        b(:,n+i)=waypoints(:,i+1);
    end
    coff=A\b';
    
end
end




