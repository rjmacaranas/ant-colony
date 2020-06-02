%----------------------------------------------------------------------------------------------
% An Investigation of Hybrid Ant Colony Optimization
% Contributors: Miclaine Emtman, RJ Macaranas, Elias Sutter
% 
% MATLAB Implementation of Dijkstra-ACO algorithm proposed by
% Nie and Zhao (2019)
%
% EE 509: Computational Intelligence
% Cal Poly, SLO
% Dr. Helen Yu, Spring 2020
%----------------------------------------------------------------------------------------------
clc;
clear;
close all;

load exampleMaps.mat                    % import example maps
map = binaryOccupancyMap(simpleMap);    % construct map

% robotRadius = 0.2;
% mapInflated = copy(map);
% inflate(mapInflated, robotRadius);
% 
% mapMatrix = ~simpleMap;                 % map indexing is flipped

image = imread('Dijkstra-aco-map.png');     % read image
bw_image = image < 0.5;                     % filter high pixels
mapMatrix = logical(bw_image);              % create logic matrix
map = binaryOccupancyMap(mapMatrix);        % create occupancy grid

image = imread('../images/r_Dijkstra-aco-map.png');
bw_image = image < 0.5;
bw_image = imresize(bw_image, 0.5);
mapMatrix = logical(bw_image);

display(mapMatrix)
startLocation = [5 5];
endLocation = [12 2];

xPos = startLocation(2);                % initialize robot x position
yPos = startLocation(1);                % initialize robot y position

xGoal = 17;
yGoal = 30;

%------------------------------------------------------------------
% The following is adapted from alexranaldi's A_STAR repository
% on GitHub which can be found at:
% https://github.com/alexranaldi/A_STAR
%
% License for use is located in our project repository
%------------------------------------------------------------------
path = [];                              % initialize path

mapDimensions = size(mapMatrix);        % get dimensions
mapSize = numel(size(mapMatrix));       % get # elements

start = sub2ind(mapDimensions, ...      % convert to linear index
        xPos, yPos);

goal = sub2ind(mapDimensions, ...
        xGoal, yGoal);
    
costs = ones(mapDimensions);             % initialize costs

openSet = false(mapDimensions);        
openSet(start) = true;

closedSet = false(mapDimensions);

previous = zeros(1, mapSize);

gScore = inf(mapDimensions);            % gScore[n]: cost of cheapest move
gScore(start) = 0;

[gr, gc] = ind2sub(mapDimensions, goal);

fScore = inf(mapDimensions);
fScore(start) = compute_cost( ...       % initialize fScore[start]
    mapDimensions, start, gr, gc);

tic
while any(openSet(:) > 0)
    
    [~, current] = min(fScore(:));
    
    if current == goal
        final = get_path(previous, current);
        toc
        a_star_plot(mapMatrix, costs, final);

        return
    end
    
    rc = rem(current - 1, mapDimensions(1)) + 1;
    col = (current - rc) / mapDimensions(1) + 1;
    
    openSet(rc, col) = false;
    closedSet(rc, col) = true;
    
    fScore(rc, col) = inf;
    gScoreCurrent = gScore(rc, col) + costs(rc, col);
    
    S2 = sqrt(2);
    
    n_ss = [ ...
            rc + 1, col + 1, S2 ;    ...
            rc + 1, col + 0, 1 ;     ...
            rc + 1, col - 1, S2 ;    ...
            rc + 0, col - 1, 1 ;     ...
            rc - 1, col - 1, S2 ;    ...
            rc - 1, col - 0, 1 ;     ... 
            rc - 1, col + 1, S2 ;    ...
            rc - 0, col + 1, 1 ;     ...
    ];

    valid_row = n_ss(:,1) >= 1 & n_ss(:,1) <= mapDimensions(1);
    valid_col = n_ss(:,2) >= 1 & n_ss(:,2) <= mapDimensions(2);
    n_ss = n_ss(valid_row & valid_col, :);
    
    neighbors = n_ss(:,1) + (n_ss(:,2) - 1) .* mapDimensions(1);
    ixInMap = mapMatrix(neighbors) & ~closedSet(neighbors);
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
