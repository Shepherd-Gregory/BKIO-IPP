function L_copy = ray_casting_simu(x,L_copy,sensor,param)
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
rmax = sensor.maxRange;
FOV = sensor.FOV;
scanAngle = sensor.scanAngle;
alpha = param.alpha;
obs_list_virtual = [];
extend_list_virtual = [];
log_free = 0.5; % 0.05 default
log_occ = 1;  % 0.1 default
log_min = -4; % -0.2 default
log_max = 4;

for x_virtual = 1:param.M
    for y_virtual = 1:param.N
        if L_copy(x_virtual, y_virtual) > 0
            obs_list_virtual = [obs_list_virtual; [x_virtual, y_virtual]];
        end
    end
end

for i = 1:length(scanAngle)
    theta = scanAngle(i);
    break_flag = 0;
    theta_m = x(3);
    
    x_v = rmax * cos(theta);
    y_v = rmax * sin(theta);
    x_m = x_v * cos(theta_m) - y_v * sin(theta_m) + x(1);
    y_m = x_v * sin(theta_m) + y_v * cos(theta_m) + x(2);
    
    x_m = min(max(x_m,1),param.M);
    y_m = min(max(y_m,1), param.N);
    
    ray_array = bresenham(x(1), x(2), x_m, y_m);
    for p = 1:length(ray_array(:,1))
        free_flag = 1;
        for obs_index = 1:length(obs_list_virtual(:,1))
            dist = ray_array(p,:) - obs_list_virtual(obs_index,:);
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
                L_copy(ray_array(p,1), ray_array(p,2)) = L_copy(ray_array(p,1), ray_array(p,2)) - log_free;
                if L_copy(ray_array(p,1), ray_array(p,2)) < log_min
                    L_copy(ray_array(p,1), ray_array(p,2)) = log_min;
                end
                extend_list_virtual = [extend_list_virtual;ray_array(p,:)];
            end
        end
        if break_flag == 1
            if ray_array(p,1) <= param.M && ray_array(p,2) <= param.N
                L_copy(ray_array(p,1), ray_array(p,2)) = L_copy(ray_array(p,1), ray_array(p,2)) + log_occ;
                extend_list_virtual = [extend_list_virtual;ray_array(p,:)];
            end
            break
        end
        
    end
end
end

