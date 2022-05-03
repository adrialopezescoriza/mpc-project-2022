%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = compute_tightening(K_tube,H_tube,h_tube,params)  
	% Initiallize polyhedrons
    X = Polyhedron('A',params.constraints.StateMatrix,'b',params.constraints.StateRHS);
    U = Polyhedron('A',params.constraints.InputMatrix,'b',params.constraints.InputRHS);
    Tube = Polyhedron('A',H_tube,'b',h_tube);

    % Modify spaces
    Xz = minus(X,Tube);
    Uz = minus(U,K_tube*Tube);

    % Adapt constraints in parameters struct
    params.constraints.StateMatrix = Xz.A;
    params.constraints.StateRHS = Xz.b;
    params.constraints.InputMatrix = Uz.A;
    params.constraints.InputRHS = Uz.b;
end