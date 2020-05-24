%------------------------------------------------------------------
% An Investigation of Hybrid Ant Colony Optimization
% Contributors: Miclaine Emtman, RJ Macaranas, Elias Sutter
% 
% Robot Running Environment Test 
%
% EE 509: Computational Intelligence
% Cal Poly, SLO
% Dr. Helen Yu, Spring 2020
%------------------------------------------------------------------

image = imread('../images/r_Dijkstra-aco-map.png');
% image = imresize(image, 0.04);
% gray_image = rgb2gray(image);
map_matrix = image < 0.5;
map = binaryOccupancyMap(~map_matrix);

show(map)
