%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE_forces
    properties
        forces_optimizer
    end

    methods
        function obj = MPC_TE_forces(Q,R,N,params)
            nu = params.model.nu;
            nx = params.model.nx;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            X0 = sdpvar(nx,1,'full');

            % define constraints
            H_x = params.constraints.StateMatrix;
            H_u = params.constraints.InputMatrix;
            h_x = params.constraints.StateRHS;
            h_u = params.constraints.InputRHS;

            [~,P_lqr] = dlqr(params.model.A, params.model.B, Q, R);

            constraints = [];
            objective = 0;
            X{1} = X0;
            for i=1:N
                objective = objective + traj_cost(X{i},U{i},Q,R);
                constraints = [constraints, H_x*X{i} <= h_x, H_u*U{i} <= h_u];
                constraints = [constraints, X{i+1} == params.model.A*X{i} + params.model.B*U{i}];        
            end

            objective = objective + X{N+1}'*P_lqr*X{N+1};
            constraints = [constraints, X{N+1}==0];

            opts = getOptions('forcesSolver');
            opts.printlevel = 0;
            obj.forces_optimizer = optimizerFORCES(constraints,objective,opts,...
                                                    X0,U{1},{'xinit'},{'u0'});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            [optimizer_out,errorcode,info] = obj.forces_optimizer(x);
            u = optimizer_out;
            objective = info.pobj;
            solvetime = info.solvetime;

            feasible = true;
            if (errorcode ~= 1)
                feasible = false;
                warning('MPC infeasible');
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end