%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints(params)
    smax = params.constraints.MaxAbsPositionXZ;
    ymax = params.constraints.MaxAbsPositionY;
    umax = params.constraints.MaxAbsThrust;

    h_x = [smax ymax smax smax ymax smax]';
    h_u = [umax umax umax umax umax umax]';

    H_x = [1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 1 0 0 0;
           -1 0 0 0 0 0;
           0 -1 0 0 0 0;
           0 0 -1 0 0 0];

    H_u = [1 0 0;
           0 1 0;
           0 0 1;
           -1 0 0;
           0 -1 0;
           0 0 -1];
end