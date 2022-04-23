%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% BRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix
% OUTPUT:
%   H, h: Describes polytopic X_LQR = {x | H * x <= h}

function [H, h] = lqr_maxPI(Q,R,params)
	% YOUR CODE HERE
    system = LTISystem('A',params.model.A,'B',params.model.B);

    system.x.penalty = QuadFunction(Q);
    system.u.penalty = QuadFunction(R);

    % Specify bject functions
    Px = Polyhedron('A',params.constraints.StateMatrix,'b',params.constraints.StateRHS);
    Pu = Polyhedron('A',params.constraints.InputMatrix,'b',params.constraints.InputRHS);

    % Specify constriant types (only polyhedra for now)
    system.x.with('setConstraint');
    system.u.with('setConstraint');

    % Set constrints
    system.x.setConstraint = Px;
    system.u.setConstraint = Pu;

    % Compute invariant set and get Polytopic form
    InvSet = system.LQRSet();
    H = InvSet.A;
    h = InvSet.b;
end

