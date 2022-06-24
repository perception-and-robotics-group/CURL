function [point_cloud_output] = outlier_filter(varargin)
    if nargin==3
        point_cloud = varargin{1};
        range = varargin{2};
        neighbor = varargin{3};
        point_cloud_kd = KDTreeSearcher(point_cloud);
        point_cloud_output = point_cloud;
        idx = rangesearch(point_cloud_kd,point_cloud,range);
        for i = 1:size(idx,1)
            if length(idx{i}) < neighbor
                point_cloud_output(i,:) = [0 0 0];
            end
        end
    elseif nargin==4
        point_cloud_1 = varargin{1};
        point_cloud_2 = varargin{2};
        range = varargin{3};
        neighbor = varargin{4};
        point_cloud_kd = KDTreeSearcher(point_cloud_1);
        point_cloud_output = point_cloud_2;
        idx = rangesearch(point_cloud_kd,point_cloud_2,range);
        for i = 1:size(idx,1)
            if length(idx{i}) < neighbor
                point_cloud_output(i,:) = [0 0 0];
            end
        end
    end
end