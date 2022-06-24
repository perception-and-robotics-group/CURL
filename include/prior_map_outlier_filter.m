function [point_cloud_output] = prior_map_outlier_filter(point_cloud,prior_map_kd,range,neighbor)
    point_cloud_output = point_cloud;
    idx = rangesearch(prior_map_kd,point_cloud,range);
    parfor i = 1:size(idx,1)
        if length(idx{i}) < neighbor
            point_cloud_output(i,:) = [0 0 0];
        end
    end
end