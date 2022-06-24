function patchConfig = patch_config_function(varargin)
% Read parameters
    for i = 1:nargin
        if strcmp(varargin{i},'degree_initial')
            patchConfig.degree_initial = varargin{i+1};
        elseif strcmp(varargin{i},'degree_iter_step')
            patchConfig.degree_iter_step = varargin{i+1};
        elseif strcmp(varargin{i},'degree_max')
            patchConfig.degree_max = varargin{i+1};
        elseif strcmp(varargin{i},'patch_row_rso')
            patchConfig.patch_row_rso = varargin{i+1}; % can use this value to calculate the lidarConfig.max_length
        elseif strcmp(varargin{i},'patch_col_rso')
            patchConfig.patch_col_rso = varargin{i+1};
        elseif strcmp(varargin{i},'row_overlap_step')
            patchConfig.row_overlap_step = varargin{i+1};
        elseif strcmp(varargin{i},'col_overlap_step')
            patchConfig.col_overlap_step = varargin{i+1};            
        elseif strcmp(varargin{i},'k')
            patchConfig.k = varargin{i+1};
        elseif strcmp(varargin{i},'is_1_1_recons_only')
            patchConfig.is_1_1_recons_only = varargin{i+1};
            if patchConfig.is_1_1_recons_only
                patchConfig.row_overlap_step = 0;
                patchConfig.col_overlap_step = 0;                
            end
        end
    end
end

