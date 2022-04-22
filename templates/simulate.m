%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,u_info] = simulate(x0, ctrl, params)

Nt = params.model.HorizonLength;

Xt = zeros(params.model.nx,Nt);
Ut = zeros(params.model.nu,Nt-1);

Xt(:,1) = x0;

for i=1:Nt
    [Ut(:,i), u_info(i)] = ctrl.eval(Xt(:,i));
    Xt(:,i+1) = params.model.A*Xt(:,i) + params.model.B*Ut(:,i);
end
end