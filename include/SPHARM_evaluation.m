function [error_dis,true_compression_percentage,double_compression_percentage] = SPHARM_evaluation(varargin)
% Read parameters NEW_COLLEGE
    if nargin == 16
        data_set = varargin{1};
        save_coefficients_dir = varargin{2};
        mask_update = varargin{3};
        point_cloud_original = varargin{4};
        reconsPointCloud = varargin{5};
        frame_num = varargin{6};
        pose_correction_dir = varargin{7};
        pcd_file_name = varargin{8};
        poses = varargin{9};
        prior_map = varargin{10};
        lidarConfig = varargin{11};
        reconsConfig = varargin{12};
        save_evaluation_error_dis_file = varargin{13};
        save_evaluation_compression_file = varargin{14};
        isSave = varargin{15};
        save_sampling_dir = varargin{16};
    elseif nargin == 13
        data_set = varargin{1};
        save_coefficients_dir = varargin{2};
        mask_update = varargin{3};
        point_cloud_original = varargin{4};
        reconsPointCloud = varargin{5};
        frame_num = varargin{6};
        pcd_file_name = varargin{7};
        lidarConfig = varargin{8};
        reconsConfig = varargin{9};
        save_evaluation_error_dis_file = varargin{10};
        save_evaluation_compression_file = varargin{11};
        isSave = varargin{12};
        save_sampling_dir = varargin{13};
    elseif nargin == 10
        data_set = varargin{1};
        save_coefficients_dir = varargin{2};
        mask_update = varargin{3};
        point_cloud_original = varargin{4};
        reconsPointCloud = varargin{5};
        frame_num = varargin{6};
        save_evaluation_error_dis_file = varargin{7};
        save_evaluation_compression_file = varargin{8};
        isSave = varargin{9};
        save_sampling_dir = varargin{10};
    elseif nargin == 11
        data_set = varargin{1};
        lidarConfig = varargin{2};
        save_coefficients_dir = varargin{3};
        mask_update = varargin{4};
        point_cloud_original = varargin{5};
        reconsPointCloud = varargin{6};
        frame_num = varargin{7};
        save_evaluation_error_dis_file = varargin{8};
        save_evaluation_compression_file = varargin{9};
        isSave = varargin{10};
        save_sampling_dir = varargin{11};
    elseif nargin == 15
        data_set = varargin{1};
        lidarConfig = varargin{2};
        save_coefficients_dir = varargin{3};
        mask_update = varargin{4};
        point_cloud_original = varargin{5};
        point_cloud_deep = varargin{6};
        points_from_mesh = varargin{7};
        reconsPointCloud = varargin{8};
        frame_num = varargin{9};
        save_deep_method_error_dis_file = varargin{10};
        save_up_sampled_error_dis_file = varargin{11};
        save_evaluation_error_dis_file = varargin{12};
        save_evaluation_compression_file = varargin{13};
        isSave = varargin{14};
        save_sampling_dir = varargin{15};
    end
    error_dis = [];
    true_compression_percentage = [];
    double_compression_percentage = [];
    if isSave == true
        if strcmp(data_set,'NEW_COLLEGE')
            if ~reconsConfig.is_density_enhance
                disp('Without Density Enhancement');
                % Calculate the error
                if ~exist(save_evaluation_error_dis_file,'file')
                    error_dis = compare_point_cloud_kd(point_cloud_original,reconsPointCloud);
                else
                    load(save_evaluation_error_dis_file);
                end
                error = mean(error_dis);
                std_error = std(error_dis);
                disp(['Frame: ' num2str(frame_num-1) ' Mean Distance Error: ' num2str(error) '(m)' ' Std: ' num2str(std_error)]);
               % Calculate the Compression percentage
                files = dir(save_coefficients_dir);
                for i = 1:size(files,1)
                    if strcmp(files(i).name,'coefficient_r_cell.mat')
                        break;
                    end
                end
                coefficients_size = files(i).bytes;
                mask_size = length(mask_update.col)*4/8;            
                files = dir(save_sampling_dir);
                for i = 1:size(files,1)
                    if strcmp(files(i).name,'true_point_cloud_original.mat')
                        break;
                    end
                end
                true_point_cloud_original_size = files(i).bytes;
                true_compression_percentage = (coefficients_size+mask_size)/true_point_cloud_original_size*100;
                disp(['Frame: ' num2str(frame_num-1) ' Mat File Compression Percentage: ' num2str(true_compression_percentage) '%']);
                % final metric by using double for the compression percentage
                load([save_sampling_dir '/true_point_cloud_original.mat']);
                load([save_coefficients_dir '/coefficient_r_cell.mat']);
                num_coefficients = 0;
                for i=1:size(coefficient_r_cell,1)
                    for j=1:size(coefficient_r_cell,2)
                        if(~isempty(coefficient_r_cell{i,j}))
                            num_coefficients = num_coefficients+length(coefficient_r_cell{i,j});
                        end
                    end
                end
                double_compression_percentage = (num_coefficients+length(mask_update.col)*4/64)/(size(true_point_cloud_original,1)*size(true_point_cloud_original,2))*100;
                disp(['Frame: ' num2str(frame_num-1) ' Double Compression Percentage: ' num2str(double_compression_percentage) '%']);
            else
                disp('With Density Enhancement');
                % Transform the prior_map
                prior_map_transformed = NEW_COLLEGE_t_lidar_w_correction(pcd_file_name,frame_num,poses,pose_correction_dir,prior_map);
                inlier_idx  = (vecnorm(prior_map_transformed')<=lidarConfig.max_length);
                point_cloud_gt = prior_map_transformed(inlier_idx,:);
                if ~exist(save_evaluation_error_dis_file,'file')
                    error_dis = compare_point_cloud_kd(point_cloud_gt,reconsPointCloud);
                else
                    load(save_evaluation_error_dis_file);
                end
                error = mean(error_dis);
                std_error = std(error_dis);
                disp(['Frame: ' num2str(frame_num-1) ' Mean Distance Error: ' num2str(error) '(m)' ' Std: ' num2str(std_error)]);
                % Calculate the Compression percentage
                files = dir(save_coefficients_dir);
                for i = 1:size(files,1)
                    if strcmp(files(i).name,'coefficient_r_cell.mat')
                        break;
                    end
                end
                coefficients_size = files(i).bytes;
                mask_size = length(mask_update.col)*4/8;            
                files = dir(save_sampling_dir);
                for i = 1:size(files,1)
                    if strcmp(files(i).name,'true_point_cloud_original.mat')
                        break;
                    end
                end
                true_point_cloud_original_size = files(i).bytes;
                true_compression_percentage = (coefficients_size+mask_size)/true_point_cloud_original_size*100;
                disp(['Frame: ' num2str(frame_num-1) ' Mat File Compression Percentage: ' num2str(true_compression_percentage) '%']);
                % final metric by using double for the compression percentage
                load([save_sampling_dir '/true_point_cloud_original.mat']);
                load([save_coefficients_dir '/coefficient_r_cell.mat']);
                num_coefficients = 0;
                for i=1:size(coefficient_r_cell,1)
                    for j=1:size(coefficient_r_cell,2)
                        if(~isempty(coefficient_r_cell{i,j}))
                            num_coefficients = num_coefficients+length(coefficient_r_cell{i,j});
                        end
                    end
                end
                double_compression_percentage = (num_coefficients+length(mask_update.col)*4/64)/(size(true_point_cloud_original,1)*size(true_point_cloud_original,2))*100;
                disp(['Frame: ' num2str(frame_num-1) ' Double Compression Percentage: ' num2str(double_compression_percentage) '%']);
            end
        elseif strcmp(data_set,'KITTI')
            % Calculate the error
            if ~exist(save_evaluation_error_dis_file,'file')
                error_dis = compare_point_cloud_kd(point_cloud_original,reconsPointCloud);
            else
                load(save_evaluation_error_dis_file);
            end
            error = mean(error_dis);
            std_error = std(error_dis);
            disp(['Frame: ' num2str(frame_num-1) ' Mean Distance Error: ' num2str(error) '(m)' ' Std: ' num2str(std_error)]);
           % Calculate the Compression percentage
            files = dir(save_coefficients_dir);
            for i = 1:size(files,1)
                if strcmp(files(i).name,'coefficient_r_cell.mat')
                    break;
                end
            end
            coefficients_size = files(i).bytes;
            mask_size = length(mask_update.col)*4/8;        
            files = dir(save_sampling_dir);
            for i = 1:size(files,1)
                if strcmp(files(i).name,'true_point_cloud_original.mat')
                    break;
                end
            end
            true_point_cloud_original_size = files(i).bytes;
            true_compression_percentage = (coefficients_size+mask_size)/true_point_cloud_original_size*100;
            disp(['Frame: ' num2str(frame_num-1) ' Mat File Compression Percentage: ' num2str(true_compression_percentage) '%']);
            % final metric by using double for the compression percentage
            load([save_sampling_dir '/true_point_cloud_original.mat']);
            load([save_coefficients_dir '/coefficient_r_cell.mat']);
            num_coefficients = 0;
            for i=1:size(coefficient_r_cell,1)
                for j=1:size(coefficient_r_cell,2)
                    if(~isempty(coefficient_r_cell{i,j}))
                        num_coefficients = num_coefficients+length(coefficient_r_cell{i,j});
                    end
                end
            end
            double_compression_percentage = (num_coefficients+length(mask_update.col)*4/64)/(size(true_point_cloud_original,1)*size(true_point_cloud_original,2))*100;
            disp(['Frame: ' num2str(frame_num-1) ' Double Compression Percentage: ' num2str(double_compression_percentage) '%']);
    
    
        elseif strcmp(data_set,'INDOOR')
            % Calculate the error
            if ~exist(save_evaluation_error_dis_file,'file')
                [~,elev,~] = cart2sph(point_cloud_original(:,1),point_cloud_original(:,2),point_cloud_original(:,3));
                idx1 = elev<=lidarConfig.fov_up;
                idx2 = elev>=lidarConfig.fov_down;
                point_cloud_original = point_cloud_original((idx1&idx2),:);
                error_dis = compare_point_cloud_kd(point_cloud_original,reconsPointCloud);
            else
                load(save_evaluation_error_dis_file);
            end
            error = mean(error_dis);
            std_error = std(error_dis);
            disp(['Frame: ' num2str(frame_num-1) ' Mean Distance Error: ' num2str(error) '(m)' ' Std: ' num2str(std_error)]);
           % Calculate the Compression percentage
            files = dir(save_coefficients_dir);
            for i = 1:size(files,1)
                if strcmp(files(i).name,'coefficient_r_cell.mat')
                    break;
                end
            end
            coefficients_size = files(i).bytes;
            mask_size = length(mask_update.col)*4/8;        
            files = dir(save_sampling_dir);
            for i = 1:size(files,1)
                if strcmp(files(i).name,'true_point_cloud_original.mat')
                    break;
                end
            end
            true_point_cloud_original_size = files(i).bytes;
            true_compression_percentage = (coefficients_size+mask_size)/true_point_cloud_original_size*100;
            disp(['Frame: ' num2str(frame_num-1) ' Mat File Compression Percentage: ' num2str(true_compression_percentage) '%']);
            % final metric by using double for the compression percentage
            load([save_sampling_dir '/true_point_cloud_original.mat']);
            load([save_coefficients_dir '/coefficient_r_cell.mat']);
            num_coefficients = 0;
            for i=1:size(coefficient_r_cell,1)
                for j=1:size(coefficient_r_cell,2)
                    if(~isempty(coefficient_r_cell{i,j}))
                        num_coefficients = num_coefficients+length(coefficient_r_cell{i,j});
                    end
                end
            end
            double_compression_percentage = (num_coefficients+length(mask_update.col)*4/64)/(size(true_point_cloud_original,1)*size(true_point_cloud_original,2))*100;
            disp(['Frame: ' num2str(frame_num-1) ' Double Compression Percentage: ' num2str(double_compression_percentage) '%']);
        elseif strcmp(data_set,'SUPER_RSO')
            % Calculate the deep method error
            if ~exist(save_deep_method_error_dis_file,'file')
                deep_method_error_dis = compare_point_cloud_kd(point_cloud_original,point_cloud_deep);
            else
                load(save_deep_method_error_dis_file);
            end
            % Calculate the up-sampling mask error
            if ~exist(save_up_sampled_error_dis_file,'file')
                up_sampled_error_dis = compare_point_cloud_kd(point_cloud_original,points_from_mesh);
            else
                load(save_up_sampled_error_dis_file);
            end
            % Calculate the recons error
            if ~exist(save_evaluation_error_dis_file,'file')           
                error_dis = compare_point_cloud_kd(point_cloud_original,reconsPointCloud);
            else
                load(save_evaluation_error_dis_file);
            end
            result_string = ['Frame: ' num2str(frame_num-1) '\n' ...
                  'Deep:        Mean Distance Error: ' num2str(mean(deep_method_error_dis)) '(m)' ' Std: ' num2str(std(deep_method_error_dis)) '\n' ...
                  'UpSampled:   Mean Distance Error: ' num2str(mean(up_sampled_error_dis)) '(m)' ' Std: ' num2str(std(up_sampled_error_dis)) '\n' ...
                  'Recons:      Mean Distance Error: ' num2str(mean(error_dis)) '(m)' ' Std: ' num2str(std(error_dis)) '\n'];
            fprintf(result_string);
           % Calculate the Compression percentage
            files = dir(save_coefficients_dir);
            for i = 1:size(files,1)
                if strcmp(files(i).name,'coefficient_r_cell.mat')
                    break;
                end
            end
            coefficients_size = files(i).bytes;
            mask_size = length(mask_update.col)*4/8;        
            files = dir(save_sampling_dir);
            for i = 1:size(files,1)
                if strcmp(files(i).name,'true_point_cloud_original.mat')
                    break;
                end
            end
            true_point_cloud_original_size = files(i).bytes;
            true_compression_percentage = (coefficients_size+mask_size)/true_point_cloud_original_size*100;
            disp(['Frame: ' num2str(frame_num-1) ' Mat File Compression Percentage: ' num2str(true_compression_percentage) '%']);
            % final metric by using double for the compression percentage
            load([save_sampling_dir '/true_point_cloud_original.mat']);
            load([save_coefficients_dir '/coefficient_r_cell.mat']);
            num_coefficients = 0;
            for i=1:size(coefficient_r_cell,1)
                for j=1:size(coefficient_r_cell,2)
                    if(~isempty(coefficient_r_cell{i,j}))
                        num_coefficients = num_coefficients+length(coefficient_r_cell{i,j});
                    end
                end
            end
            double_compression_percentage = (num_coefficients+length(mask_update.col)*4/64)/(size(true_point_cloud_original,1)*size(true_point_cloud_original,2))*100;
            disp(['Frame: ' num2str(frame_num-1) ' Double Compression Percentage: ' num2str(double_compression_percentage) '%']);
        end
        
        save(save_evaluation_error_dis_file,'error_dis');
        save([save_coefficients_dir '/true_compression_percentage.mat'],'true_compression_percentage');
        save([save_coefficients_dir '/double_compression_percentage.mat'],'double_compression_percentage');
        if exist('deep_method_error_dis','var')
            save(save_deep_method_error_dis_file,'deep_method_error_dis');
        end
        if exist('up_sampled_error_dis','var')
            save(save_up_sampled_error_dis_file,'up_sampled_error_dis');
        end
    else        
        disp('Evaluation needs isSave=true');
    end
end