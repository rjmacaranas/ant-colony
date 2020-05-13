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

%------------------------------------------------------------------
% The following is adapted from alexranaldi's A_STAR repository
% on GitHub which can be found at:
% https://github.com/alexranaldi/A_STAR
%------------------------------------------------------------------

path = [];                              % initialize path
mapDimensions = size(map);              % get dimensions
mapSize = numel(size(map));             % get # elements

openSet = false(mapDimension);        
openSet(xPos, yPos) = true;

closedSet = false(mapDimension);

previous = zeros(1, mapSize);

gScore = inf(mapDimensions);            % gScore[n]: cost of cheapest move
gScore(xPos, yPos) = 0;

[gr, gc] = ind2sub(madDimensions, endLocation);

fScore = inf(mapDimensions);
fScore(xPos, yPos)  % need to insert cost function here

while any(openSet(:) > 0)
    
    [~, current] = min(fScore(:));
    
    if current == goal
        final = get_path(previous, current);
        return
    end
    
    row = mod(current - 1, mapDimensions(1)) + 1;
    col = (current - row) / mapDimensions(1) + 1;
    
    openSet(row, col) = false;
    closedSet(row, col) = true;
    
    fScore(row, col) = inf;
    gScoreCurrent = gScore(row, col) + costs(row, col);
    
    S2 = sqrt(2);
    
    n_ss = [ ...
            rc + 1, cc + 1, S2 ; ...
            rc + 1, cc + 0, 1 ; ...
            rc + 1, cc - 1, S2 ; ...
            rc + 0, cc - 1, 1 ; ...
            rc - 1, cc - 1, S2 ; ...
            rc - 1, cc - 0, 1 ; ... 
            rc - 1, cc + 1, S2 ; ...
            rc - 0, cc + 1, 1 ; ...
    ];

    valid_row = n_ss(:,1) >= 1 & n_ss(:,1) <= mapDimensions(1);
    valid_col = n_ss(:,2) >= 1 & n_ss(:,2) <= mapDimensions(2);
    n_ss = n_ss(valid_row & valid_col, :);
    
    neighbors = n_ss(:,1) + (n_ss(:,2) - 1) .* mapDimensions(1);
    ixInMap = map(neighbors) & ~closedSet(neighbors);
    neighbors = neighbors(ixInMap);
    
    dists = n_ss(ixInMap, 3);
    
    openSet(neighbors) = true;
    
    tentative_gscores = gScoreCurrent + costs(neighbors) .* dists;
    
    ixBetter = tentative_gscores < gScore(neighbors);
    bestNeighbors = neighbors(ixBetter);
    
    previous(bestNeighbors) = current;
    gScore(bestNeighbors) = tentative_gscores(ixBetter);
    fScore(bestNeighbors) = gScore(bestNeighbors) + ...
                    compute_cost(mapDimensions, bestNeighbors, gr, gc);
    
end

function p = get_path(previous, current)
    % Returns the path. This function is only called once and therefore
    %   does not need to be extraordinarily efficient
    inds = find(previous);
    p = nan(1, length(inds));
    p(1) = current;
    next = 1;
    while any(current == inds)
        current = previous(current);
        next = next + 1;
        p(next) = current; 
    end
    p(isnan(p)) = [];
end


function cost = compute_cost(mapSize, from, rTo, cTo)
    % Returns COST, an estimated cost to travel the map, starting FROM and
    %   ending at TO.
    [rFrom,cFrom] = ind2sub(mapSize, from);
    cost = sqrt((rFrom - rTo).^2 + (cFrom - cTo).^2);
end

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