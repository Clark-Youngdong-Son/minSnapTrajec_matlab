%% main
clear all; close all;

parameters;                      %parameters initialization

if(keyframe_selection_flag)
    keyframe_selection;
else
    load('keyframe.mat','keyframe');
end

QP_trajec_generation;

drawSolution;