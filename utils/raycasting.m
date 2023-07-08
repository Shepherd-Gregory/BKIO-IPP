function [cur_extend_list, L] = raycasting(x, sensor, obs_list, extend_list, L, param, step)
%{  
    Copyright (C) 2021  Yang Xu (xuyangxtu@foxmail.com)
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details. 
%}
% raycasting and log-odds update
rmax = sensor.maxRange;
FOV = sensor.FOV;
scanAngle = sensor.scanAngle;
alpha = param.alpha;
cur_extend_list = [];
log_free = 0.5; 
log_occ = 1;  
log_min = -4; 
log_max = 4;

for i = 1:length(scanAngle)
    theta = scanAngle(i);
    break_flag = 0;
    theta_m = x(3,step);
    
    x_v = rmax * cos(theta);
    y_v = rmax * sin(theta);
    x_m = x_v * cos(theta_m) - y_v * sin(theta_m) + x(1, step);
    y_m = x_v * sin(theta_m) + y_v * cos(theta_m) + x(2, step);
    x_m = min(max(x_m,1),param.M);
    y_m = min(max(y_m,1), param.N);
    ray_array = bresenham(x(1,step), x(2,step), x_m, y_m); 
    for p = 1:length(ray_array(:,1))
        free_flag = 1;
        for obs_index = 1:length(obs_list(:,1))
            dist = ray_array(p,:) - obs_list(obs_index,:);
            if norm(dist,2) > alpha - 0.1
                continue
            else
                break_flag = 1;
                free_flag = 0;
                break
            end
        end
            % max range case
        if free_flag == 1
            if ray_array(p,1) <= param.M && ray_array(p,2) <= param.N
                L(ray_array(p,1), ray_array(p,2)) = L(ray_array(p,1), ray_array(p,2)) - log_free; 
                if L(ray_array(p,1), ray_array(p,2)) < log_min % origin=-0.2
                    L(ray_array(p,1), ray_array(p,2)) = log_min;
                end
                cur_extend_list = [cur_extend_list;ray_array(p,:)];
                extend_list = [extend_list;ray_array(p,:)];
            end
        end
        if break_flag == 1
            if ray_array(p,1) <= param.M && ray_array(p,2) <= param.N
                L(ray_array(p,1), ray_array(p,2)) = L(ray_array(p,1), ray_array(p,2)) + log_occ;
                cur_extend_list = [cur_extend_list;ray_array(p,:)];
                extend_list = [extend_list;ray_array(p,:)];
            end
            break
        end
        
    end
end
extend_list = unique(extend_list,'rows');
cur_extend_list = unique(cur_extend_list,'rows'); 
end

