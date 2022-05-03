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

            sys = LTISystem('A',params.model.A,'B',params.model.B);

            % define constraints
            H_x = params.constraints.StateMatrix;
            H_u = params.constraints.InputMatrix;
            h_x = params.constraints.StateRHS;
            h_u = params.constraints.InputRHS;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            EPS = sdpvar(repmat(size(h_x,1),1,N+1),ones(1,N+1),'full');
            X = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            X0 = sdpvar(nx,1,'full');

            [~,P_lqr] = dlqr(params.model.A, params.model.B, Q, R);

            constraints = [];
            objective = 0;
            X{1} = X0;
            for i=1:N
                objective = objective + traj_cost(X{i},U{i},Q,R) + EPS{i}'*S*EPS{i} + v*max(abs(EPS{i}));
                constraints = [constraints, H_x*X{i} <= (h_x+EPS{i}), H_u*U{i} <= h_u, EPS{i} >= 0];
                constraints = [constraints, X{i+1} == params.model.A*X{i} + params.model.B*U{i}];
            end

            objective = objective + X{N+1}'*P_lqr*X{N+1} + EPS{N+1}'*S*EPS{N+1} + v*max(abs(EPS{N+1}));
            % Belongs to feasible set
            constraints = [constraints, H*X{N+1} <= h, H_x*X{N+1} <= (h_x+EPS{N+1}), EPS{N+1} >= 0];

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