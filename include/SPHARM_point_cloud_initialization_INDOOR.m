function [square_grid,polar_form] = SPHARM_point_cloud_initialization_INDOOR(Location,lidarConfig)
    is_INDOOR = true;
    [polar_form.yaw, polar_form.pitch, polar_form.r] = cart2sph(Location(:,1),Location(:,2),Location(:,3));
    % normalizing and Scaling
    u_angle = @(u) lidarConfig.full_fov*(1-u/lidarConfig.full_num_lasers)-abs(lidarConfig.fov_down);
    lidarConfig.u_begin_inc = pi/2 - u_angle(1);
    lidarConfig.u_end_inc = pi/2 - u_angle(lidarConfig.full_num_lasers);
    lidarConfig.u_res = abs(lidarConfig.u_begin_inc-lidarConfig.u_end_inc)/lidarConfig.full_num_lasers;
    pic.u = round((lidarConfig.full_num_lasers-1)*(1-(polar_form.pitch - lidarConfig.fov_down)/lidarConfig.full_fov))+1;
    idx_u_lower = pic.u < 1;
    idx_u_larger = pic.u > lidarConfig.full_num_lasers;
    if is_INDOOR == false
        if sum(idx_u_lower)>0
            pic.u(idx_u_lower) = 1;
        end
        if sum(idx_u_larger)>0
            pic.u(idx_u_larger) = lidarConfig.full_num_lasers;
        end
    else
        idx_u_null = idx_u_lower|idx_u_larger;
    end
    v_angle = @(v) 2*pi*(v/lidarConfig.img_length-0.5);
    lidarConfig.v_begin = v_angle(1);
    lidarConfig.v_end = v_angle(lidarConfig.img_length);
    lidarConfig.v_res = 2*pi/1024;
    pic.v = round(lidarConfig.img_length*(0.5*((polar_form.yaw/pi) + 1)))+1;
    idx_v_lower = pic.v<1;
    idx_v_larger = pic.v>lidarConfig.img_length;
    if sum(idx_v_lower)>0
        pic.v(idx_v_lower) = lidarConfig.img_length;
    end
    if sum(idx_v_larger)>0
        pic.v(idx_v_larger) = 1;
    end    
    pic.r = polar_form.r;
    pic.u = int16(pic.u);
    pic.v = int16(pic.v);
    if is_INDOOR == true
        idx_null = idx_u_null;
        if sum(idx_null)>0
            pic.r(idx_null) = [];
            pic.u(idx_null) = [];
            pic.v(idx_null) = [];
        end
    end
    square_grid.full_grid = zeros(lidarConfig.full_num_lasers,lidarConfig.img_length);
    for n = 1:length(pic.v)
        square_grid.full_grid(pic.u(n),pic.v(n)) = pic.r(n);
    end
    
    if lidarConfig.num_lasers < lidarConfig.full_num_lasers        
        square_grid.grid = square_grid.full_grid(lidarConfig.low_res_index,:);                
    else
        square_grid.grid = square_grid.full_grid;
    end
    square_grid.grid(square_grid.grid<lidarConfig.min_length) = 0;
    [table,~,~] = table_generation(lidarConfig,0);
    [square_grid.point_cloud(:,1), square_grid.point_cloud(:,2), square_grid.point_cloud(:,3)] = sph2cart(table.yaw, table.pitch, reshape(square_grid.grid',numel(square_grid.grid),1));
    square_grid.r = reshape(square_grid.grid',numel(square_grid.grid),1);
end

