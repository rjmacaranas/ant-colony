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

