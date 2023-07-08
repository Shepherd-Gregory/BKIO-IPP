function MI_update_list = action_MI(candidate_list, L, Hm_0, sensor, param)
%{
    Copyright (C) 2022  Yang Xu (xuyangxtu@foxmail.com)
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
%}
    MI_update_list = [];
    for i = 1:length(candidate_list(:,1))
        candi = candidate_list(i,:);
        L_copy = L;
        temp_x = candi';
        L_copy_updated = ray_casting_simu(temp_x,L_copy,sensor,param);
        
        P_copy_updated = exp(L_copy_updated)./(ones(param.M,param.N) + exp(L_copy_updated));

        [~, HM_new] = cal_MI(P_copy_updated);
        MI_update = Hm_0 - HM_new;
        MI_update_list = [MI_update_list; MI_update];
    end
end