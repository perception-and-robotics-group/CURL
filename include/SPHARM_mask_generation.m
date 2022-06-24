function [mask,non_zero_data] = SPHARM_mask_generation(num_lasers,img_length,square_grid,thresholdConfig,max_length,min_length)
    mask.grid = zeros(num_lasers,img_length);
    non_zero_data.grid = zeros(num_lasers,img_length);
    non_zero_data.allpoints = zeros(size(square_grid.point_cloud));
    count = 1;
    for i = 1:num_lasers
        for j = 1:img_length
            if (square_grid.r((i-1)*img_length+j) > (min_length)) && (square_grid.r((i-1)*img_length+j) < (max_length))
                mask.grid(i,j) = 1;
                non_zero_data.grid(i,j) = square_grid.grid(i,j);
                non_zero_data.points(count,:) = square_grid.point_cloud((i-1)*img_length+j,:);
                non_zero_data.allpoints((i-1)*img_length+j,:) = square_grid.point_cloud((i-1)*img_length+j,:);
                non_zero_data.r(count,:) = square_grid.r((i-1)*img_length+j,:);
                non_zero_data.index(count,:) = [i,j];
                count=count+1;
            end
        end
    end

    mask.cliff_u = zeros(num_lasers,img_length);
    mask.cliff_u(1,:) = 1;
    mask.cliff_v = zeros(num_lasers,img_length);
    mask.cliff_d = zeros(num_lasers,img_length);
    for i = 1:num_lasers
        for j = 1:img_length
            i_nei = i-1;
            j_nei = j-1;
            if(i_nei<1)
                i_nei = 1;
            end
            if(j_nei<1)
                j_nei = img_length;
            end
            mask.cliff_u(i,j) = (distance_vertex(square_grid.point_cloud((i-1)*img_length+j,:),square_grid.point_cloud((i_nei-1)*img_length+j,:))>thresholdConfig.u);
            mask.cliff_v(i,j) = (distance_vertex(square_grid.point_cloud((i-1)*img_length+j,:),square_grid.point_cloud((i-1)*img_length+j_nei,:))>thresholdConfig.v);
            mask.cliff_d(i,j) = (distance_vertex(square_grid.point_cloud((i-1)*img_length+j,:),square_grid.point_cloud((i_nei-1)*img_length+j_nei,:))>thresholdConfig.d);
        end
    end
    mask.col = reshape(mask.grid',numel(mask.grid),1);
end

