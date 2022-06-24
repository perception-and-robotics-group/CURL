function prior_map_sample = prior_map_initialation_function(prior_map,poses,lidarConfig)
    [~,pose_idx] = min(vecnorm((poses(:,1:2)-pcd_time(:,1:2))')); % Compare the time stamp and corresponding index
    t_w_base = trans_matrix(poses(pose_idx,3:5),poses(pose_idx,6:9));
    % transform prior map
    t_base_lidar = [-0.703850920000000	-0.710042360000000	  0.0208262400000000	-0.0708875842553600;
                     0.709921630000000	-0.704141670000000	 -0.0139926900000000	-0.00329640329251000;
                     0.0246000300000000	 0.00493623000000000  0.999685190000000	     0.0484381967759900;
                     0	                 0	                  0 	                 1];
    icp_correct_for_1_second = [   0.999969 -0.00416021  0.00662754   0.0431538;
                                    0.00397639    0.999614   0.0275118    -0.12978;
                                    -0.00673944  -0.0274846      0.9996    0.187711;
                                    0           0           0           1];
    prior_map_transformed = inv(icp_correct_for_1_second*t_w_base*t_base_lidar)*[prior_map ones(length(prior_map),1)]';
    prior_map_transformed = prior_map_transformed(1:3,:)';
    inlier_idx  = (vecnorm(prior_map_transformed')<=lidarConfig.max_length);
    prior_map_sample = prior_map_transformed(inlier_idx,:);
end

