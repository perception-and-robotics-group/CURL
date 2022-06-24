function [table,table_grid,table_SPHARM_grid] = table_generation(config,is_density_enhance)
    count = 1;
    if (is_density_enhance)
        for u = 1:(config.row_times*config.num_lasers)
            for v = 1:(config.col_times*config.img_length)
                table.yaw(count,1) = ((2*(v-1))/(config.col_times*config.img_length)-1)*pi;
                table.pitch(count,1) = (config.row_times*config.num_lasers-u)*(config.fov/(config.num_lasers-1)/config.row_times)+config.fov_down;
                count = count+1;
            end
        end
    else
        for u = 1:config.num_lasers
            for v = 1:config.img_length
                table.yaw(count,1) = ((2*(v-1))/config.img_length-1)*pi;
                table.pitch(count,1) = (config.num_lasers-u)*(config.fov/(config.num_lasers-1))+config.fov_down;
%                 table.pitch(count,1) = config.fov*(1-(u-1)/(config.num_lasers-1)) + config.fov_down;
                count = count+1;
            end
        end
    end
    table_grid = [table.yaw table.pitch];
    table_SPHARM_grid = [pi+table.yaw pi/2-table.pitch];
end
% (config.fov+config.fov/((config.num_lasers-1)*config.row_times))*(1-(u-1)/(config.row_times*config.num_lasers-1))+config.fov_down
% config.fov*(1-(u-1)/(row_times*config.num_lasers-1)) + config.fov_down
% ((2*(v-1))/(col_times*config.img_length)-1)*pi
% 
% (row_times*config.num_lasers-u)*(config.fov/(config.num_lasers-1)/row_times)+config.fov_down;