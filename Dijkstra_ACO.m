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

load exampleMaps.mat                    % import example maps
map = binaryOccupancyMap(simpleMap, 2); % construct map

%------------------------------------------------------------------
% The following is a test using Probabilistic Roadmap (PRM)
% to become familiar with path planning in the Mobile Robotic 
% Algorithms section of the MATLAB Robotics System Toolbox
% ** Will remove for the final submission
%------------------------------------------------------------------

robotRadius = 0.2;
mapInflated = copy(map);
inflate(mapInflated, robotRadius);

prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 50;

startLocation = [2 1];
endLocation = [12 2];

path = findpath(prm, startLocation, endLocation);
show(prm);