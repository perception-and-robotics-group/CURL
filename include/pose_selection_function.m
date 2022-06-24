function [pcd_frames_selected,pose_selected,position_selected] = pose_selection_function(data_set,pcd_dir_files,pose_file,pose_select_step)
    pcd_frames_selected = [];
    pose_selected = [];
    position_selected = [];
    if strcmp(data_set,'NEW_COLLEGE')
        poses = readmatrix(pose_file);
        poses(:,2) = poses(:,2).*10^(-6);
        for i = 1:size(pcd_dir_files,1)
            if i == 1
                pcd_time = [str2num(pcd_dir_files(i).name(7:16)) str2num(pcd_dir_files(i).name(18:26))*10^(-6)];
                [~,pose_idx] = min(vecnorm((poses(:,1:2)-pcd_time(:,1:2))'));
                position = poses(pose_idx,3:5);
                pcd_frames_selected(end+1,:) = i;
                pose_selected(end+1,:) = pose_idx;
                position_selected(end+1,:) = position;
            else
                pcd_time = [str2num(pcd_dir_files(i).name(7:16)) str2num(pcd_dir_files(i).name(18:26))*10^(-6)];
                [~,pose_idx] = min(vecnorm((poses(:,1:2)-pcd_time(:,1:2))'));
                position = poses(pose_idx,3:5);
                min_distance = min(vecnorm([position_selected-position]'));
                if min_distance > pose_select_step
                    pcd_frames_selected(end+1,:) = i;
                    pose_selected(end+1,:) = pose_idx;
                    position_selected(end+1,:) = position;
                end
            end
        end
    elseif strcmp(data_set,'KITTI')
        poses = load(pose_file);
        for i = 1:size(poses,1)
            if i == 1
                position = [poses(i,4) poses(i,8) poses(i,12)];
                pcd_frames_selected(end+1,:) = i;
                position_selected(end+1,:) = position;
            else
                position = [poses(i,4) poses(i,8) poses(i,12)];
                min_distance = min(vecnorm([position_selected-position]'));
                if min_distance > pose_select_step
                    pcd_frames_selected(end+1,:) = i;
                    position_selected(end+1,:) = position;
                end
            end
        end
    end
end