function thresholdConfig = threshold_config_function(varargin)
% Read parameters
    for i = 1:nargin
        if strcmp(varargin{i},'data_set')
            data_set = varargin{i+1};
        elseif strcmp(varargin{i},'avg_filter_thres')
            thresholdConfig.avg_filter_thres = varargin{i+1};
        elseif strcmp(varargin{i},'rmv_filter_thres')
            thresholdConfig.rmv_filter_thres = varargin{i+1};
        elseif strcmp(varargin{i},'avg_error_thres_patch')
            thresholdConfig.avg_error_thres_patch = varargin{i+1};
        elseif strcmp(varargin{i},'remove')
            thresholdConfig.remove = varargin{i+1};
        elseif strcmp(varargin{i},'u')
            thresholdConfig.u = varargin{i+1};
        elseif strcmp(varargin{i},'v')
            thresholdConfig.v = varargin{i+1};
        end
    end
    if exist('data_set','var')
        if strcmp(data_set,'NEW_COLLEGE')
            if ~isfield(thresholdConfig,'u')
                thresholdConfig.u = 5;
            end
            if ~isfield(thresholdConfig,'v')
                thresholdConfig.v = 2;
            end
        elseif strcmp(data_set,'KITTI')
            if ~isfield(thresholdConfig,'u')
                thresholdConfig.u = 2;
            end
            if ~isfield(thresholdConfig,'v')
                thresholdConfig.v = 0.2;
            end
        elseif strcmp(data_set,'INDOOR')
            % 0.1 0.1 0.2 0.2 0.5 0.5
            if ~isfield(thresholdConfig,'u')
                thresholdConfig.u = 0.1;
            end
            if ~isfield(thresholdConfig,'v')
                thresholdConfig.v = 0.1;
            end
        elseif strcmp(data_set,'SUPER_RSO')
            if ~isfield(thresholdConfig,'u')
                thresholdConfig.u = 5;
            end
            if ~isfield(thresholdConfig,'v')
                thresholdConfig.v = 2;
            end
        end
    end
    thresholdConfig.d = norm([thresholdConfig.u thresholdConfig.v]);
end

