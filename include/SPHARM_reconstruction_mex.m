function [reconsPointCloud, reconsPointCloudFiltered] = SPHARM_reconstruction_mex(varargin)
    % Read parameters
    lidarConfig = varargin{1}; % must read from the final version
    coefficient_r_cell = varargin{2};
    mask_update = varargin{3};
    reconsConfig = varargin{4};
    saveReconsDir = varargin{5};
    isSave = varargin{6};
    isFigure = varargin{7};
    % Files initialization
    % reconstruction point cloud
    reconsPointCloud_file = [saveReconsDir '/reconsPointCloud.mat'];
    reconsPointCloudFiltered_file = [saveReconsDir '/reconsPointCloudFiltered.mat'];
    % Procedure
    if ~exist(reconsPointCloud_file, 'file')
        [~, ~, table_SPHARM_grid] = table_generation_manual(lidarConfig, reconsConfig.row_times, reconsConfig.col_times);               
        mask_new_directly = SPHARM_mask_recons_generation_manual_enhance(mask_update, reconsConfig.row_times, reconsConfig.col_times, lidarConfig.num_lasers, lidarConfig.img_length);
        [reconstruction_time,reconsPointCloud,reconsPointCloudFiltered] = CURL_reconstruction_mex(lidarConfig,reconsConfig,coefficient_r_cell,table_SPHARM_grid,mask_new_directly.grid);
        disp(['Reconstruction Time: ' num2str(reconstruction_time) '(s)']);               
    else
        load(reconsPointCloud_file);
        load(reconsPointCloudFiltered_file);
    end

    if isSave == true

        if ~exist(saveReconsDir, 'dir')
            mkdir(saveReconsDir);
        end

        save(reconsPointCloud_file, 'reconsPointCloud');
        save(reconsPointCloudFiltered_file, 'reconsPointCloudFiltered');
        % save reconstruction time
        if exist('reconstruction_time','var')
            save([saveReconsDir '/reconstruction_time.mat'],'reconstruction_time');
        end
    end

    if isFigure == true
        fig = figure('Name', ['SPHARM Reconstruction' saveReconsDir]);
        % reconstruction figure
        figure(fig);
        subplot(1, 2, 1);
        pcshow(reconsPointCloud);
        title('Reconstruction Point Cloud');
        subplot(1, 2, 2);
        pcshow(reconsPointCloudFiltered);
        title('Reconstruction Point Cloud Remove Outliers');
    end

end
