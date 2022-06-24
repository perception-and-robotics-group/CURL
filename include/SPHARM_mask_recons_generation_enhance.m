function reons_mask_grid = SPHARM_mask_recons_generation_enhance(mask,config)
    approx_threshold = 0.01;
    reons_mask_grid.grid = zeros(config.row_times*config.num_lasers,config.col_times*config.img_length);
    for j = 1:size(reons_mask_grid.grid,2)
        upper_flag = true;
        for i = 1:size(reons_mask_grid.grid,1)
            u = ceil(double(i)/config.row_times);
            u_true = round(double(i)/config.row_times);
            u_decimal = u-double(i)/config.row_times;
            if u < 1
                u = 1;
            elseif u > config.num_lasers
                u = config.num_lasers;
            end
            if u_true < 1
                u_true = 1;
            elseif u_true > config.num_lasers
                u_true = config.num_lasers;
            end
            v = ceil(double(j+config.col_times-1)/config.col_times);
            v_true = round(double(j+config.col_times-1)/config.col_times);
            v_decimal = v-double(j+config.col_times-1)/config.col_times;
            if v < 1
                v = 1;
            elseif v > config.img_length
                v = 1;
            end
            if v_true < 1
                v_true = config.img_length;
            elseif v_true > config.img_length
                v_true = 1;
            end
            if(u_decimal==0&&mask.grid(u,v_true)==1)
                upper_flag = false;
            end
            if(upper_flag==false)
                reons_mask_grid.grid(i,j) = mask.grid(u_true,v_true);
                if u == 1
                    if (mask.cliff_u(u,v)==1) && (u_decimal > approx_threshold)
                        reons_mask_grid.grid(i,j) = 0;
                    end
                    if (mask.cliff_v(u,v) == 1) && (v_decimal > approx_threshold) && (v_decimal < (1-approx_threshold))
                        reons_mask_grid.grid(i,j) = 0;
                    end

                    if (mask.cliff_d(u,v) == 1) && (u_decimal > approx_threshold) && (v_decimal > approx_threshold) && (v_decimal < (1-approx_threshold))
                        reons_mask_grid.grid(i,j) = 0;
                    end
                else
                    if (mask.cliff_u(u,v)==1) && (u_decimal > approx_threshold) && (u_decimal < (1-approx_threshold))
                        reons_mask_grid.grid(i,j) = 0;
                    end
                    if (mask.cliff_v(u,v) == 1) && (v_decimal > approx_threshold) && (v_decimal < (1-approx_threshold))
                        reons_mask_grid.grid(i,j) = 0;
                    end

                    if (mask.cliff_d(u,v) == 1) && (((u_decimal > approx_threshold) && (u_decimal < (1-approx_threshold)))||((v_decimal > approx_threshold) && (v_decimal < (1-approx_threshold))))
                        reons_mask_grid.grid(i,j) = 0;
                    end
                end
            end
        end
    end
    reons_mask_grid.col = reshape(reons_mask_grid.grid',numel(reons_mask_grid.grid),1);
    clear mask config;
end

