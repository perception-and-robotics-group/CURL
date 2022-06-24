function lidarConfig = lidar_config_function(varargin)
% Read parameters
    for i = 1:nargin
        if strcmp(varargin{i},'data_set')
            data_set = varargin{i+1};
        elseif strcmp(varargin{i},'num_lasers')
            lidarConfig.num_lasers = varargin{i+1};
        elseif strcmp(varargin{i},'full_num_lasers')
            lidarConfig.full_num_lasers = varargin{i+1};
        elseif strcmp(varargin{i},'img_length')
            lidarConfig.img_length = varargin{i+1};
        elseif strcmp(varargin{i},'length_percentage')
            lidarConfig.length_percentage = varargin{i+1}; % can use this value to calculate the lidarConfig.max_length
        elseif strcmp(varargin{i},'max_length')
            lidarConfig.max_length = varargin{i+1};
        elseif strcmp(varargin{i},'min_length')
            lidarConfig.min_length = varargin{i+1};
        elseif strcmp(varargin{i},'row_times')
            lidarConfig.row_times = varargin{i+1};
        elseif strcmp(varargin{i},'col_times')
            lidarConfig.col_times = varargin{i+1};
        elseif strcmp(varargin{i},'full_fov_up')
            lidarConfig.full_fov_up = varargin{i+1};
        elseif strcmp(varargin{i},'fov_down')
            lidarConfig.fov_down = varargin{i+1};
        end
    end
    if exist('data_set','var')
        if strcmp(data_set,'NEW_COLLEGE')
            lidarConfig.full_fov_up = 17.743*pi/180;
            lidarConfig.fov_down = -15.594*pi/180;
            if ~isfield(lidarConfig,'length_percentage')
                lidarConfig.length_percentage = 1;
            end
            if ~isfield(lidarConfig,'min_length')
                lidarConfig.min_length = 3;
            end
        elseif strcmp(data_set,'KITTI')
            lidarConfig.full_fov_up = 2*pi/180;
            lidarConfig.fov_down = -24.9*pi/180;
            if ~isfield(lidarConfig,'length_percentage')
                lidarConfig.length_percentage = 0.9;
            end
            if ~isfield(lidarConfig,'min_length')
                lidarConfig.min_length = 5;
            end
        elseif strcmp(data_set,'INDOOR')
            lidarConfig.full_fov_up = 17.743*pi/180;
            lidarConfig.fov_down = -15.594*pi/180;
            if ~isfield(lidarConfig,'length_percentage')
                lidarConfig.length_percentage = 1;
            end
            if ~isfield(lidarConfig,'min_length')
                lidarConfig.min_length = 1;
            end
        elseif strcmp(data_set,'SUPER_RSO')
            lidarConfig.full_fov_up = 16.6*pi/180;
            lidarConfig.fov_down = -16.6*pi/180;
            if ~isfield(lidarConfig,'length_percentage')
                lidarConfig.length_percentage = 1;
            end
            if ~isfield(lidarConfig,'min_length')
                lidarConfig.min_length = 3;
%                 lidarConfig.min_length = 0;
            end
        end
    end
    lidarConfig.full_fov = lidarConfig.full_fov_up-lidarConfig.fov_down;
    lidarConfig.u_res = lidarConfig.full_fov/(lidarConfig.full_num_lasers-1);
    if lidarConfig.num_lasers < lidarConfig.full_num_lasers
        lidarConfig.upscaling_factor = lidarConfig.full_num_lasers / lidarConfig.num_lasers;
        lidarConfig.low_res_index = flip(lidarConfig.full_num_lasers:(-lidarConfig.upscaling_factor):1);
        min_idx = min(lidarConfig.low_res_index);        
        lidarConfig.fov = lidarConfig.u_res*(lidarConfig.full_num_lasers-min_idx);
        lidarConfig.fov_up = lidarConfig.fov+lidarConfig.fov_down;
    else
        lidarConfig.fov = lidarConfig.full_fov;
        lidarConfig.fov_up = lidarConfig.full_fov_up;
    end
    if(lidarConfig.row_times==1)&&(lidarConfig.col_times==1)
        lidarConfig.is_density_enhance = 0;
    else
        lidarConfig.is_density_enhance = 1;
    end
end

