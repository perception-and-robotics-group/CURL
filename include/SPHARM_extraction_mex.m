function [lidarConfig,coefficient_r_cell,mask_update,true_point_cloud_original,square_grid,points_from_mesh,Extraction_time] = SPHARM_extraction_mex(varargin)
% Read parameters
    frame_num = varargin{1};
    inputPointCloud = varargin{2};
    lidarConfig = varargin{3};
    thresholdConfig= varargin{4};
    patchConfig = varargin{5};
    saveSamplingDir = varargin{6};
    saveSamplingRemoveDir = varargin{7};
    saveCoefficientsDir = varargin{8};
    isSave = varargin{9};
    isFigure = varargin{10};
% Files initialization
    % lidarConfig file
    lidarConfig_file = [saveSamplingDir '/lidarConfig.mat'];
    % kd_filtered_cloud file
    kd_filtered_cloud_file = [saveSamplingDir '/kd_filtered_cloud.mat'];
    % true ground truth point cloud
    true_point_cloud_original_file = [saveSamplingDir '/true_point_cloud_original.mat'];
    true_point_cloud_original_ply_file = [saveSamplingDir '/true_point_cloud_original.ply'];
    % sampled points before remove outliers
    original_sampled_file = [saveSamplingDir '/sampled_points.mat'];    
    % sampled points after remove outliers
    sampled_file = [saveSamplingRemoveDir '/points_from_mesh.mat'];
    % all sampled points after remove outliers
    sampled_all_file = [saveSamplingRemoveDir '/points_from_mesh_all.mat'];
    % coefficients cells
    coefficient_r_cell_file = [saveCoefficientsDir '/coefficient_r_cell.mat'];
    % mask_recons
    mask_recons_file = [saveCoefficientsDir '/mask_recons.mat'];
    % mask after updating
    mask_update_file = [saveCoefficientsDir '/mask_update.mat'];
    % original point clouds after remove outliers
    point_cloud_original_mask_update_file = [saveCoefficientsDir '/point_cloud_original_mask_update.mat'];
% Function initialization
    w = @(e,k) 1./(1+(e./k).^2);
% Point Cloud initialization
% Procedure
    inputPointCloud = outlier_filter(inputPointCloud,1,7); % nearest neighbor filter
    kd_filtered_cloud = inputPointCloud;
    % generate the mask
    [square_grid,~] = SPHARM_point_cloud_initialization(inputPointCloud,lidarConfig); % revise this function to make the input has lidarConfig
    [mask,non_zero_data] = SPHARM_mask_generation(lidarConfig.num_lasers,lidarConfig.img_length,square_grid,thresholdConfig,lidarConfig.max_length,lidarConfig.min_length); % revise remove normal_value
    true_point_cloud_original = non_zero_data.points;
    lidarConfig.max_length = max(vecnorm(true_point_cloud_original'));
    % generate the mesh
    [~,faces,~] = mesh_generation(square_grid.point_cloud, mask, lidarConfig.img_length);
    
    %  generate the table for up-sampling and for SPHARM coefficient
    [~, table_init_grid, table_init_SPHARM_grid] = table_generation(lidarConfig,lidarConfig.is_density_enhance);
    recons_table_check = table_validity_check(lidarConfig);
    if(lidarConfig.is_density_enhance)
        mask_recons = SPHARM_mask_recons_generation_enhance(mask,lidarConfig);
        % make the preparation for the mask update
        mask_recons.update_grid = mask_recons.grid; 
    else
        mask_recons = mask;
        % make the preparation for the mask update
        mask_recons.update_grid = mask_recons.grid;
    end
    % up-sampling
    if (lidarConfig.is_density_enhance)
        if ~exist(original_sampled_file,'file')
            [sampled_points, ~, ~,~,ray_tracing_time] = intersection_between_faces_lines_grid_mex(non_zero_data,faces,table_init_grid,recons_table_check);            
        else
            load(original_sampled_file);
        end        
        if (~exist(sampled_file,'file'))||(~exist(sampled_all_file,'file'))
            points_from_mesh = sampled_points;
            count = 1;
            mask.sample_grid = zeros(size(sampled_points,1),1);
            mid_points_from_mesh = [];        
            points_from_mesh_all.point_cloud = zeros(size(sampled_points,1),3); % The reason we need "points_from_mesh_all" is because we need it to generate nw mask after upsampling
            for i = 1:size(sampled_points,1)
                if  (mask_recons.col(i,:)~=0) && ((points_from_mesh(i,1) ~= 0)||(points_from_mesh(i,2) ~= 0)||(points_from_mesh(i,3) ~= 0))
                    mid_points_from_mesh(count,:) = points_from_mesh(i,:);
                    points_from_mesh_all.point_cloud(i,:) = points_from_mesh(i,:);
                    mask.sample_grid(i,:) = 1;
                    count = count+1;
                end
            end
            points_from_mesh = mid_points_from_mesh;
        else
            load(sampled_file);
            load(sampled_all_file);
        end
    else
        sampled_points = non_zero_data.points;
        points_from_mesh_all.point_cloud = non_zero_data.allpoints;
        points_from_mesh = non_zero_data.points;
    end    
    
    if (~exist(coefficient_r_cell_file,'file')||(~exist(mask_recons_file,'file')))
        [Extraction_time,coefficient_r_cell,mask_recons.update_grid] = CURL_extraction_mex(lidarConfig,patchConfig,thresholdConfig,points_from_mesh_all.point_cloud ,table_init_SPHARM_grid,mask_recons.update_grid);
        disp(['Actual Extraction Time: ' num2str(Extraction_time) '(s)']);
    else
        load(coefficient_r_cell_file);
        load(mask_recons_file);
    end
    if ~exist(mask_update_file,'file')
        % update the original mask
        % still has a little bug
        mask_update = mask;
        for i = 1:size(mask_recons.update_grid,1)
            for j = 1:size(mask_recons.update_grid,2)          
                u = ceil(double(i)/lidarConfig.row_times);
                if u < 1
                    u = 1;
                elseif u > lidarConfig.num_lasers
                    u = lidarConfig.num_lasers;
                end
                v = ceil(double(j+lidarConfig.col_times-1)/lidarConfig.col_times);               
                if v < 1
                    v = lidarConfig.img_length;
                elseif v > lidarConfig.img_length
                    v = 1;
                end
                if ((u-double(i)/lidarConfig.row_times) == 0 )&& ((v - double(j+lidarConfig.col_times-1)/lidarConfig.col_times) == 0)
                    mask_update.grid(u,v) = mask_recons.update_grid(i,j);
                end
                if (mask_recons.update_grid(i,j) == 0) && ((u-double(i)/lidarConfig.row_times) ~= 0) && ((double(j+lidarConfig.col_times-1)/lidarConfig.col_times-v) == 0)
                    mask_update.cliff_u(u,v) = 1;
                end
                if (mask_recons.update_grid(i,j) == 0) && ((u-double(i)/lidarConfig.row_times) == 0) && ((double(j+lidarConfig.col_times-1)/lidarConfig.col_times-v) ~= 0)
                    mask_update.cliff_v(u,v) = 1;
                end
                if (mask_recons.update_grid(i,j) == 0) && ((u-double(i)/lidarConfig.row_times) ~= 0) && ((double(j+lidarConfig.col_times-1)/lidarConfig.col_times-v) ~= 0)
                    mask_update.cliff_d(u,v) = 1;
                end
            end
        end
        mask_update.col = reshape(mask_update.grid',numel(mask_update.grid),1);
    else
        load(mask_update_file);
    end    
    

    % update the original point cloud after appling the mask_update
    point_cloud_original_mask_update = non_zero_data.allpoints(mask_update.col>0,:);
    if isSave == true
        % create folders
        if ~exist(saveSamplingDir,'dir')
            mkdir(saveSamplingDir);
        end
        if ~exist(saveSamplingRemoveDir,'dir')
            mkdir(saveSamplingRemoveDir);
        end
        if ~exist(saveCoefficientsDir,'dir')
            mkdir(saveCoefficientsDir);
        end
        % lidarConfig file
        save(lidarConfig_file,'lidarConfig');
        % kd_filtered_cloud file
        save(kd_filtered_cloud_file,'kd_filtered_cloud');
        % true ground truth point cloud    
        save(true_point_cloud_original_file,'true_point_cloud_original');
        % PLY file    
        pt_true_point_cloud_original = pointCloud(single(true_point_cloud_original));
        pcwrite(pt_true_point_cloud_original,true_point_cloud_original_ply_file,'PLYFormat','ascii');
        % sampled points before remove outliers
        save(original_sampled_file,'sampled_points');
        if exist('ray_tracing_time','var')
            % save ray-tracing time
            save([saveSamplingDir '/ray_tracing_time.mat'],'ray_tracing_time');
        end
        % sampled points after remove outliers
        save(sampled_file,'points_from_mesh');
        % all sampled points after remove outliers
        save(sampled_all_file,'points_from_mesh_all');
        % coefficients cells
        save(coefficient_r_cell_file,'coefficient_r_cell');
        % save extration time
        if exist('Extraction_time','var')
            save([saveCoefficientsDir '/Extraction_time.mat'],'Extraction_time')
        end
        
        % mask_recons
        save(mask_recons_file,'mask_recons');
        % mask after updating
        save(mask_update_file,'mask_update');
         % original point clouds after remove outliers
        save(point_cloud_original_mask_update_file,'point_cloud_original_mask_update');
    end
    % show figures
    if isFigure == true
        fig = figure('Name','SPHARM Extraction Sampling');
        % original figure
        figure(fig);
        subplot(3,2,1);
        pcshow(inputPointCloud);
        title('Before remove remote points');
        % removed outlier points
        figure(fig);
        subplot(3,2,2);
        pcshow(true_point_cloud_original)
        title('After remove remote points')
        % mesh figure
        subplot(3,2,3);
        trisurf(faces,non_zero_data.points(:,1),non_zero_data.points(:,2),non_zero_data.points(:,3));
        title('Mesh');
        % original sampled point cloud
        subplot(3,2,4);
        pcshow(sampled_points);
        title('Nosied Sampled Point Cloud');
        % sampled point cloud remove outlier
        subplot(3,2,5);
        pcshow(points_from_mesh);
        title('Denoised Point Cloud');
        % comparsion between original sampled point cloud and original point cloud
        fig_new = figure('Name','SPHARM Extraction Sampling Comparsion');
        pt_original = pointCloud(non_zero_data.points);
        pt_sampled = pointCloud(sampled_points);
        subplot(1,2,1);
        pcshowpair(pt_sampled,pt_original);
        title('Noised Point Cloud');
        % comparsion between sampled point cloud and original point cloud
        pt_points_from_mesh = pointCloud(points_from_mesh);
        subplot(1,2,2);
        pcshowpair(pt_points_from_mesh,pt_original);
        title('Denoised Point Cloud');
        disp(['mask grid size shoule be: ' num2str(sum(mask_recons.update_grid,'all'))]);
        % mask figures
        figure('Name','Mask should be');
        imshow(mask_recons.update_grid,[]);            
    end
end