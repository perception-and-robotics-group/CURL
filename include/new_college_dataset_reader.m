function [point_cloud,prior_map_transformed,t_w_base] = new_college_dataset_reader(frame_number,pcd_folder,pose_file,prior_map)
    % read pcd file
    pcd_files = dir(pcd_folder);
    pcd_files(1:2,:) = [];
    pcd_time = [str2num(pcd_files(frame_number).name(7:16)) str2num(pcd_files(frame_number).name(18:26))*10^(-6)]; % read the time stamp from the name of the pcd file
    point_cloud = pcread([pcd_folder pcd_files(frame_number).name]);
    % read pose file
    poses = readmatrix(pose_file);
    poses(:,2) = poses(:,2).*10^(-6); 
    [~,pose_idx] = min(vecnorm((poses(:,1:2)-pcd_time(:,1:2))')); % Compare the time stamp and corresponding index
    t_w_base = trans_matrix(poses(pose_idx,3:5),poses(pose_idx,6:9)); % trans_matrix is a function can transfer "translation" and "quaterion" into "transformation matrix"
    % transform prior map
    t_base_lidar = [-0.703850920000000	-0.710042360000000	  0.0208262400000000	-0.0708875842553600;
                     0.709921630000000	-0.704141670000000	 -0.0139926900000000	-0.00329640329251000;
                     0.0246000300000000	 0.00493623000000000  0.999685190000000	     0.0484381967759900;
                     0	                 0	                  0 	                 1];
    
    icp_correct_for_1 = [0.999982 -0.00488706   0.0035492   0.0954628;
                     0.00479999    0.999697   0.0241389   -0.116827;
                     -0.00366609  -0.0241214    0.999702    0.208053;
                     0           0           0           1];
    icp_correct_for_1_second = [   0.999969 -0.00416021  0.00662754   0.0431538;
                                    0.00397639    0.999614   0.0275118    -0.12978;
                                    -0.00673944  -0.0274846      0.9996    0.187711;
                                    0           0           0           1];
    prior_map_transformed = inv(icp_correct_for_1_second*t_w_base*t_base_lidar)*[prior_map ones(length(prior_map),1)]';
    prior_map_transformed = prior_map_transformed(1:3,:)';
end

