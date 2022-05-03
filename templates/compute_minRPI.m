%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params)
    % Initialize things
    sys = LTISystem('A',params.model.A+params.model.B*K_tube);
    n_iter = 0;

    % Constrin polyhedrons
    Px = Polyhedron('A',params.constraints.StateMatrix,'b',params.constraints.StateRHS);
    Pu_x = Polyhedron('A',params.constraints.InputMatrix*(K_tube),'b',params.constraints.InputRHS);
    Px = intersect(Px,Pu_x);
    Pw = Polyhedron('A',params.constraints.DisturbanceMatrix,'b',params.constraints.DisturbanceRHS);

    % Initialize invariant set
    R = Px;
    R_prev = R;

    while(true)
        preR = sys.reachableSet('X',minus(R,Pw),'direction','backward'); % Compute preset
        R = intersect(R,preR); % Intersect set and preset
        if (eq(R.minHRep(),R_prev.minHRep))
            break;
        end
        R_prev = R;
        n_iter = n_iter + 1;
    end

    H_tube = R.A;
    h_tube = R.b;
end