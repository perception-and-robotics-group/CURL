function reconsConfig = recons_config_function(varargin)
% Read parameters
    for i = 1:nargin
        if strcmp(varargin{i},'recons_row_times')
            reconsConfig.row_times = varargin{i+1};
        elseif strcmp(varargin{i},'recons_col_times')
            reconsConfig.col_times = varargin{i+1};
        elseif strcmp(varargin{i},'lidarConfig')
            lidarConfig = varargin{i+1};
        end
    end
    reconsConfig.is_density_enhance = 0;
    if reconsConfig.col_times > 1
        reconsConfig.is_density_enhance = 1;
    elseif ~exist('lidarConfig','var')
        if reconsConfig.row_times > 1
            reconsConfig.is_density_enhance = 1;
            disp('Attention: There is no lidarConfig as input, but defined as continuous reconstruction');
        end
    elseif exist('lidarConfig','var')
        if (reconsConfig.row_times*lidarConfig.num_lasers) > lidarConfig.full_num_lasers
            reconsConfig.is_density_enhance = 1;
        end
    end
end

