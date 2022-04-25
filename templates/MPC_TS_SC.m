%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC(Q,R,N,H,h,S,v,params)   
            nu = params.model.nu;
            nx = params.model.nx;

            % define constraints
            H_x = params.constraints.StateMatrix;
            H_u = params.constraints.InputMatrix;
            h_x = params.constraints.StateRHS;
            h_u = params.constraints.InputRHS;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            EPS = sdpvar(repmat(size(h_x,1),1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full');

            [~,P_lqr] = dlqr(params.model.A, params.model.B, Q, R);

            constraints = [];
            objective = 0;
            x = X0;
            for i=1:N
                objective = objective + traj_cost(x,U{i},Q,R) + EPS{i}'*S*EPS{i} + v * max(EPS{i});
                constraints = [constraints, H_x*x <= (h_x+EPS{i}), H_u*U{i} <= h_u, EPS{i} >= 0];
                x = params.model.A*x + params.model.B*U{i};        
            end

            objective = objective + x'*P_lqr*x;
            % Belongs to feasible set
            constraints = [constraints, H*x <= h];

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end