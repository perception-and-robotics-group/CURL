function [table,table_grid,table_SPHARM_grid] = table_generation_manual(config,recons_row_times,recons_col_times)
    count = 1;
%     if (recons_row_times == 1) && (recons_col_times==1)
%         for u = 1:(config.num_lasers)
%             for v = 1:(config.img_length)
%                 table.yaw(count,1) = ((2*(v-1))/config.img_length-1)*pi;
%                 table.pitch(count,1) = config.fov*(1-(u-1)/(config.num_lasers-1)) + config.fov_down;
%                 count = count+1;
%             end
%         end
%     else
        for u = 1:(recons_row_times*config.num_lasers)
            for v = 1:(recons_col_times*config.img_length)
                table.yaw(count,1) = ((2*(v-1))/(recons_col_times*config.img_length)-1)*pi;
                table.pitch(count,1) = (recons_row_times*config.num_lasers-u)*(config.fov/(config.num_lasers-1)/recons_row_times)+config.fov_down;
                count = count+1;
            end
        end
        table_grid = [table.yaw table.pitch];
        table_SPHARM_grid = [pi+table.yaw pi/2-table.pitch];
%     end
end

