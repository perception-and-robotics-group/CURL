function [config,square_grid,polar_form] = point_cloud_initialization(Location,num_lasers,img_length,row_times,col_times,normal_value)
    [polar_form.yaw, polar_form.pitch, polar_form.r] = cart2sph(Location(:,1),Location(:,2),Location(:,3));
%     parameters
%     config.fov_up = max(polar_form.pitch);
%     config.fov_down = min(polar_form.pitch);
%     new college data set
    config.fov_up = 17.743*pi/180;
    config.fov_down = -15.594*pi/180;
    % kitti parameters
%     config.fov_up = 2*pi/180;
%     config.fov_down = -24.9*pi/180;
%     config.fov_down = -28*pi/180;
    config.num_lasers = num_lasers;
    config.img_length = img_length;
    config.fov = config.fov_up + abs(config.fov_down);
    config.row_times = row_times;
    config.col_times = col_times;
    config.normal_value = normal_value;
    % normalizing and Scaling
    u_angle = @(u) config.fov*(1-u/config.num_lasers)-abs(config.fov_down);
    config.u_begin_inc = pi/2 - u_angle(1);
    config.u_end_inc = pi/2 - u_angle(config.num_lasers);
    config.u_res = abs(config.u_begin_inc-config.u_end_inc)/config.num_lasers;

%     pic.u = ceil(config.num_lasers*(1 - (polar_form.pitch + abs(config.fov_down))/config.fov));
    pic.u = round((config.num_lasers-1)*(1-(polar_form.pitch - config.fov_down)/config.fov))+1;

    for n = 1:length(pic.u)
        if pic.u(n) <= 0
            pic.u(n) = 1;
        end
        if pic.u(n) >= config.num_lasers
            pic.u(n) = config.num_lasers;
        end
    end

    v_angle = @(v) 2*pi*(v/config.img_length-0.5);
    config.v_begin = v_angle(1);
    config.v_end = v_angle(config.img_length);
    config.v_res = 2*pi/1024;
%     config.v_res = abs(config.v_begin-config.v_end)/config.img_length;

%     pic.v = ceil(config.img_length*(0.5*((polar_form.yaw/pi) + 1)));
    pic.v = round(config.img_length*(0.5*((polar_form.yaw/pi) + 1)))+1;
    for n = 1:length(pic.v)
        if pic.v(n) <= 0
            pic.v(n) = 1;
        end
        if pic.v(n) > config.img_length
            pic.v(n) = 1;
        end
    end
    pic.r = polar_form.r;
    pic.u = int16(pic.u);
    pic.v = int16(pic.v);

    % generate 64*1024 grid and its value
    square_grid.grid = zeros(config.num_lasers,config.img_length);
    for n = 1:length(pic.v)
        square_grid.grid(pic.u(n),pic.v(n)) = pic.r(n);
    end
   
    if(config.row_times==1)&&(config.col_times==1)
        config.is_density_enhance = 0;
    else
        config.is_density_enhance = 1;
    end
    
    [table,~,~] = table_generation(config,0);
    [square_grid.point_cloud(:,1), square_grid.point_cloud(:,2), square_grid.point_cloud(:,3)] = sph2cart(table.yaw, table.pitch, reshape(square_grid.grid',numel(square_grid.grid),1));
    square_grid.r = reshape(square_grid.grid',numel(square_grid.grid),1);
end

