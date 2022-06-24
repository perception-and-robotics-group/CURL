function [inputPointCloud,lidarConfig,point_cloud_gt] = point_cloud_initial_function(varargin)
    if nargin == 9
        % read pararmeters
        data_set = varargin{1};
        lidarConfig = varargin{2};
        reconsConfig = varargin{3};
        pt_file = varargin{4};
        pcd_file_name = varargin{5};
        frame_num = varargin{6};
        poses = varargin{7};
        pose_correction_dir = varargin{8};
        prior_map = varargin{9};
        
        % procedure
        ptCloud = pcread(pt_file);
        inputPointCloud(:,1) = reshape(ptCloud.Location(:,:,1),numel(ptCloud.Location(:,:,1)),1);
        inputPointCloud(:,2) = reshape(ptCloud.Location(:,:,2),numel(ptCloud.Location(:,:,2)),1);
        inputPointCloud(:,3) = reshape(ptCloud.Location(:,:,3),numel(ptCloud.Location(:,:,3)),1);
        inputPointCloud = double(inputPointCloud);
        if ~isfield(lidarConfig,'max_length')
            % remove outliers of points based on the percentage of the distance
            Location_r = vecnorm(inputPointCloud');
            [N,edges]= histcounts(Location_r);
            for i=1:size(N,2)
                percentage = sum(N(1:i))/length(Location_r);
                if percentage >= lidarConfig.length_percentage
                    lidarConfig.max_length = edges(i);
                    break;
                end
            end
        end
        prior_map_transformed = NEW_COLLEGE_t_lidar_w_correction(pcd_file_name,frame_num,poses,pose_correction_dir,prior_map);
        inlier_idx  = (vecnorm(prior_map_transformed')<=lidarConfig.max_length)&(vecnorm(prior_map_transformed')>0);
        point_cloud_gt = prior_map_transformed(inlier_idx,:);
        
        pt_point_cloud_gt = pointCloud(point_cloud_gt);
        pt_inputPointCloud = pointCloud(inputPointCloud);                       
        inputPointCloud_out = outlier_filter(point_cloud_gt,inputPointCloud,0.2,1);
        inputPointCloud = inputPointCloud_out;
    elseif nargin == 6
        % read pararmeters
        data_set = varargin{1};
        lidarConfig = varargin{2};
        reconsConfig = varargin{3};
        pt_file = varargin{4};
        pcd_file_name = varargin{5};
        frame_num = varargin{6};     
        % procedure
        point_cloud_gt = [];
        ptCloud = pcread(pt_file);
        inputPointCloud(:,1) = reshape(ptCloud.Location(:,:,1),numel(ptCloud.Location(:,:,1)),1);
        inputPointCloud(:,2) = reshape(ptCloud.Location(:,:,2),numel(ptCloud.Location(:,:,2)),1);
        inputPointCloud(:,3) = reshape(ptCloud.Location(:,:,3),numel(ptCloud.Location(:,:,3)),1);
        inputPointCloud = double(inputPointCloud);
        if ~isfield(lidarConfig,'max_length')
            % remove outliers of points based on the percentage of the distance
            Location_r = vecnorm(inputPointCloud');
            [N,edges]= histcounts(Location_r);
            for i=1:size(N,2)
                percentage = sum(N(1:i))/length(Location_r);
                if percentage >= lidarConfig.length_percentage
                    lidarConfig.max_length = edges(i+1);
                    break;
                end
            end
        end
    elseif nargin == 3
        data_set = varargin{1};
        lidarConfig = varargin{2};
        pt_file = varargin{3};
        point_cloud_gt = [];
        if strcmp(data_set,'KITTI')
            ptCloud = pcread(pt_file);
            inputPointCloud = double(ptCloud.Location);
            if ~isfield(lidarConfig,'max_length')
                % remove outliers of points based on the percentage of the distance
                Location_r = vecnorm(inputPointCloud');
                [N,edges]= histcounts(Location_r);
                for i=1:size(N,2)
                    percentage = sum(N(1:i))/length(Location_r);
                    if percentage >= lidarConfig.length_percentage
                        lidarConfig.max_length = edges(i);
                        break;
                    end
                end
            end
        elseif strcmp(data_set,'INDOOR')
            ptCloud = pcread(pt_file);
            inputPointCloud = double(ptCloud.Location-mean(ptCloud.Location));
            % remove outliers of points based on the percentage of the distance
            Location_r = vecnorm(inputPointCloud');
            [N,edges]= histcounts(Location_r);
            for i=1:size(N,2)
                percentage = sum(N(1:i))/length(Location_r);
                if percentage >= lidarConfig.length_percentage
                    lidarConfig.max_length = edges(i);
                    break;
                end
            end
        elseif strcmp(data_set,'SUPER_RSO')
            inputPointCloud = pt_file;
            % remove outliers of points based on the percentage of the distance
            Location_r = vecnorm(inputPointCloud');
            [N,edges]= histcounts(Location_r);
            for i=1:size(N,2)
                percentage = sum(N(1:i))/length(Location_r);
                if percentage >= lidarConfig.length_percentage
                    lidarConfig.max_length = edges(i);
                    break;
                end
            end
        end
    end
end