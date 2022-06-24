function [point_cloud_full,point_cloud_deep,point_cloud_small,lidarConfig] = preprocessing_function(NPY_image_full,deep_image_full_noise,lidarConfig)
    NPY_image_full = double(squeeze(NPY_image_full(1,:,:)));
    deep_image_full_noise = double(squeeze(deep_image_full_noise(1,:,:,:)));
    %-------------Parameters from lidar_super_resolution method------------
    is_figure = false;
    image_rows_low = 16; % 8, 16, 32
    image_rows_high = 64; % 16, 32, 64
    image_rows_full = 64;
    image_cols = 1024;
    upscaling_factor = image_rows_high / image_rows_low;
    if image_rows_full==64
        ang_res_x = 360.0/image_cols; % horizontal resolution
        ang_res_y = 33.2/(image_rows_high-1); % vertical resolution
        ang_start_y = 16.6; % bottom beam angle
        sensor_noise = 0.03;
        max_range = 80.0;
        min_range = 3.0;
    end
    normalize_ratio = 100.0;
    %-------------Process full original point cloud------------------------
    NPY_image_full(NPY_image_full>max_range|NPY_image_full<min_range) = 0;
    rwoList = zeros(size(NPY_image_full));
    colList = zeros(size(NPY_image_full));
    for i=1:image_rows_high
        rowList(i,:) = ones(1,image_cols)*i;
        colList(i,:) = 1:image_cols;
    end
    verticalAngle_full = ((rowList-1)*ang_res_y-ang_start_y)*pi/180;
    horizonAngle_full = (((colList-1)+1-(image_cols/2))*ang_res_x+90)*pi/180;
    verticalAngle_col_full = reshape(verticalAngle_full',numel(verticalAngle_full),1) ;
    horizonAngle_col_full = reshape(horizonAngle_full',numel(horizonAngle_full),1);
    r_full = reshape(NPY_image_full',numel(NPY_image_full),1);
    [point_cloud_full(:,2),point_cloud_full(:,1),point_cloud_full(:,3)] = sph2cart(horizonAngle_col_full,verticalAngle_col_full,r_full);
    point_cloud_full(:,2) = -point_cloud_full(:,2);
    idx_full = vecnorm(point_cloud_full')>0;
    point_cloud_full = point_cloud_full(idx_full,:);
    %-------------Extract samller point cloud from original----------------
    low_res_index = 1:upscaling_factor:image_rows_high;
    NPY_image_small = NPY_image_full(low_res_index,:);
    verticalAngle_small = verticalAngle_full(low_res_index,:);
    horizonAngle_small = horizonAngle_full(low_res_index,:);    
    verticalAngle_col_small = reshape(verticalAngle_small',numel(verticalAngle_small),1) ;
    horizonAngle_col_small = reshape(horizonAngle_small',numel(horizonAngle_small),1);
    r_small = reshape(NPY_image_small',numel(NPY_image_small),1);
    [point_cloud_small(:,2),point_cloud_small(:,1),point_cloud_small(:,3)] = sph2cart(horizonAngle_col_small,verticalAngle_col_small,r_small);
    point_cloud_small(:,2) = -point_cloud_small(:,2);     
    lidarConfig.fov_up = max(verticalAngle_col_small);
    lidarConfig.fov = lidarConfig.fov_up-lidarConfig.fov_down;
    idx_small = vecnorm(point_cloud_small')>0;
    point_cloud_small = point_cloud_small(idx_small,:);
    %-------------Reduce noise and produce point cloud from deep method----
    deep_image_full_noise = deep_image_full_noise*normalize_ratio;
    predImagesNoiseReduced = deep_image_full_noise(:,:,1);
    deep_image_full_noise(low_res_index,:,1) = NPY_image_full(low_res_index,:);
    % remove noise
    if length(size(deep_image_full_noise))==3 && size(deep_image_full_noise,3)==2
        noiseLables = deep_image_full_noise(:,:,2);
        predImagesNoiseReduced(noiseLables>(predImagesNoiseReduced*sensor_noise)) = 0;
        predImagesNoiseReduced(low_res_index,:) = NPY_image_full(low_res_index,:);            
    end
    predImagesNoiseReduced(predImagesNoiseReduced>max_range|predImagesNoiseReduced<min_range) = 0;
    r_deep = reshape(predImagesNoiseReduced',numel(predImagesNoiseReduced),1);
    [point_cloud_deep(:,2),point_cloud_deep(:,1),point_cloud_deep(:,3)] = sph2cart(horizonAngle_col_full,verticalAngle_col_full,r_deep);
    point_cloud_deep(:,2) = -point_cloud_deep(:,2);
    idx_deep = vecnorm(point_cloud_deep')>0;
    point_cloud_deep = point_cloud_deep(idx_deep,:);
    if is_figure == true
        pt_small = pointCloud(point_cloud_small);
        pt_full = pointCloud(point_cloud_full);
        figure('Name','small and full');
        pcshowpair(pt_full,pt_small);
        pt_deep = pointCloud(point_cloud_deep);
        figure('Name','deep and full');
        pcshowpair(pt_deep,pt_full);
    end
end

