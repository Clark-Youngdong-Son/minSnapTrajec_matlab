%% Parameters
%System
m_q = 4;                                %mass of a quadrotor
I_q = diag([0.3 0.3 0.3]);              %moment of inertia of a quadrotor
g = 9.81;                               %gravitational acceleration

%Trajectory
global m;
n = 4;                                  %number of flat outputs (x, y, z, psi)
t_f = 10;                               %final time of the simulation

keyframe_selection_flag = true;         %true : select manually, false : use the given test data
if(keyframe_selection_flag)
    m = 5;                              %number of keyframes
end

order = 6;                              %order of polynomial functions

time_interval_selection_flag = true;    %true : fixed time interval, false : optimal time interval
if(time_interval_selection_flag)
   t = linspace(0,t_f,m+1); 
end

n_intermediate = 5;
corridor_width = 0.05;
corridor_position = [3 4];              %from the i keyframe to the j keyframe