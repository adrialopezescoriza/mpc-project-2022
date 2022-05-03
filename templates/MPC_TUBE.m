%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params)
            obj.K_tube = K_tube;

            nu = params.model.nu;
            nx = params.model.nx;

            % define constraints
            H_z = params.constraints.StateMatrix;
            H_v = params.constraints.InputMatrix;
            h_z = params.constraints.StateRHS;
            h_v = params.constraints.InputRHS;

            % define optimization variables
            V = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            Z = sdpvar(repmat(nx,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full');

            [~,P_lqr] = dlqr(params.model.A, params.model.B, Q, R);

            % Constrin for initial condition
            constraints = [H_tube*(Z{1}-X0) <= h_tube];
            objective = 0;

            for i=1:N
                objective = objective + traj_cost(Z{i},V{i},Q,R);
                constraints = [constraints, H_z*Z{i} <= h_z, H_v*V{i} <= h_v];
                Z{i+1} = params.model.A*Z{i} + params.model.B*V{i};
            end

            objective = objective + Z{N+1}'*P_lqr*Z{N+1};
            % Belongs to feasible set
            constraints = [constraints, H_N*Z{N+1} <= h_N, H_z*Z{N+1} <= h_z];

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{V{1} Z{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;

            [v, z, objective] = optimizer_out{:};
            u = v + obj.K_tube*(x-z);
            
            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end