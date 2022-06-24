function point_cloud_out = NEW_COLLEGE_t_w_lidar(pcd_file,poses,point_cloud)
    pcd_time = [str2double(pcd_file(7:16)) str2double(pcd_file(18:26))*10^(-6)];
    [~,pose_idx] = min(vecnorm((poses(:,1:2)-pcd_time(:,1:2))')); % Compare the time stamp and corresponding index
    t_w_base = trans_matrix(poses(pose_idx,3:5),poses(pose_idx,6:9));
    t_base_lidar = [-0.703850920000000	-0.710042360000000	  0.0208262400000000	-0.0708875842553600;
                     0.709921630000000	-0.704141670000000	 -0.0139926900000000	-0.00329640329251000;
                     0.0246000300000000	 0.00493623000000000  0.999685190000000	     0.0484381967759900;
                     0	                 0	                  0 	                 1];
    icp_correction = [   0.999969 -0.00416021  0.00662754   0.0431538;
                                    0.00397639    0.999614   0.0275118    -0.12978;
                                    -0.00673944  -0.0274846      0.9996    0.187711;
                                    0           0           0           1];
    point_cloud_out = icp_correction*t_w_base*t_base_lidar*[point_cloud ones(length(point_cloud),1)]';
    point_cloud_out = point_cloud_out(1:3,:)';
end

