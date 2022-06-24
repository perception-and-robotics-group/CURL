function final_distance = compare_point_cloud_kd(point_cloud_original,point_cloud_recons)
    point_cloud_original_kd = KDTreeSearcher(point_cloud_original);
    [~,final_distance] = knnsearch(point_cloud_original_kd,point_cloud_recons);
end

