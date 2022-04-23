%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    xt = x(1,:);
    yt = x(2,:);
    zt = x(3,:);

    x_max = max(abs(xt));
    z_max = max(abs(zt));
    y_max = max(abs(yt));
    u_max = max(max(abs(u)));
    s_max = max(x_max, z_max);

    J_u = 0;
    for ut = u
        J_u = J_u + norm(ut)^2;
    end

    df_max = norm([xt(end),yt(end),zt(end)]);
    vf_max = norm([x(4,end),x(5,end),x(6,end)]);

    traj_feas = true;
    if s_max > params.constraints.MaxAbsPositionXZ 
        traj_feas = false;
        return;
    end

    if y_max > params.constraints.MaxAbsPositionY
        traj_feas = false;
        return;
    end

    if u_max > params.constraints.MaxAbsThrust
        traj_feas = false;
        return;
    end

    if df_max > params.constraints.MaxFinalPosDiff
        traj_feas = false;
        return;
    end

    if vf_max > params.constraints.MaxFinalVelDiff
        traj_feas = false;
        return;
    end
end

