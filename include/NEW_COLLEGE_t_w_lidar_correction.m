function point_cloud_out = NEW_COLLEGE_t_w_lidar_correction(pcd_file_name,frame_num,poses,pose_correction_dir,point_cloud)
%     pose_correction = load([pose_correction_dir '/' num2str(frame_num-1) '/transformationMatrix.txt']);
    pose_correction = load([pose_correction_dir '/' num2str(frame_num-1) '/test_complete_transfo.txt']);
    pcd_time = [str2double(pcd_file_name(7:16)) str2double(pcd_file_name(18:26))*10^(-6)];
    [~,pose_idx] = min(vecnorm((poses(:,1:2)-pcd_time(:,1:2))')); % Compare the time stamp and corresponding index
    t_w_base = trans_matrix(poses(pose_idx,3:5),poses(pose_idx,6:9));
    t_base_lidar = [-0.703850920000000	-0.710042360000000	  0.0208262400000000	-0.0708875842553600;
                     0.709921630000000	-0.704141670000000	 -0.0139926900000000	-0.00329640329251000;
                     0.0246000300000000	 0.00493623000000000  0.999685190000000	     0.0484381967759900;
                     0	                 0	                  0 	                 1];
    
    point_cloud_out = pose_correction*t_w_base*t_base_lidar*[point_cloud ones(length(point_cloud),1)]';
    point_cloud_out = point_cloud_out(1:3,:)';
end

