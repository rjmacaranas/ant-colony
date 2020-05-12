%------------------------------------------------------------------
% An Investigation of Hybrid Ant Colony Optimization
% Contributors: Miclaine Emtman, RJ Macaranas, Elias Sutter
% 
% MATLAB Implementation of Dijkstra-ACO algorithm proposed by
% Nie and Zhao (2019)
%
% EE 509: Computational Intelligence
% Cal Poly, SLO
% Dr. Helen Yu, Spring 2020
%------------------------------------------------------------------
clc;
clear;
close all;

load exampleMaps.mat                    % import example maps
map = binaryOccupancyMap(simpleMap);    % construct map

robotRadius = 0.2;
mapInflated = copy(map);
inflate(mapInflated, robotRadius);

mapMatrix = logical(flip(simpleMap));   % map indexing is flipped
startLocation = [2 1];
endLocation = [12 2];

xPos = startLocation(2);                % initialize robot x position
yPos = startLocation(1);                % initialize robot y position

path = [];                              % initialize path
mapDimensions = size(map);              % get dimensions
mapSize = numel(size(map));             % get # elements

openSet = false(mapDimension);        
openSet(xPos, yPos) = true;

closedSet = false(mapDimension);

previous = zeros(1, mapSize);

gScore = inf(mapDimensions);            % gScore[n]: cost of cheapest
gScore(xPos, yPos) = 0;

[gr, gc] = ind2sub(madDimensions, endLocation);

fScore = inf(mapDimensions);
fScore(xPos, yPos); % need to insert cost function here

% while any(openSet(:) > 0)               % while openSet is not empty
%     current = min(fScore);


% function path = reconstruct_path(prev, current)
%     total_path = [current]
%------------------------------------------------------------------
% The following is a test using Probabilistic Roadmap (PRM)
% to become familiar with path planning in the Mobile Robotic 
% Algorithms section of the MATLAB Robotics System Toolbox
% ** Will remove for the final submission
%------------------------------------------------------------------
% prm = mobileRobotPRM;
% prm.Map = mapInflated;
% prm.NumNodes = 50;
% 
% startLocation = [2 1];
% endLocation = [12 2];
% 
% path = findpath(prm, startLocation, endLocation);
% show(prm);