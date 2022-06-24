function point_cloud = NEW_COLLEGE_cloud_generation(pcd_file)   
    ptCloud = pcread(pcd_file);
    point_cloud(:,1) = reshape(ptCloud.Location(:,:,1),numel(ptCloud.Location(:,:,1)),1);
    point_cloud(:,2) = reshape(ptCloud.Location(:,:,2),numel(ptCloud.Location(:,:,2)),1);
    point_cloud(:,3) = reshape(ptCloud.Location(:,:,3),numel(ptCloud.Location(:,:,3)),1);
end

