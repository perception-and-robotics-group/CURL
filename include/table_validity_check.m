function recons_table_check = table_validity_check(config)
    recons_table_check.grid = zeros(config.row_times*config.num_lasers,config.col_times*config.img_length);
    for i = 1:size(recons_table_check.grid,1)
        for j = 1:size(recons_table_check.grid,2)
            if (mod(i,config.row_times) == 0) && (mod(j+1,config.col_times) == 0)
                recons_table_check.grid(i,j) = 1;
            end
        end
    end
    recons_table_check.col = reshape(recons_table_check.grid',numel(recons_table_check.grid),1);
end