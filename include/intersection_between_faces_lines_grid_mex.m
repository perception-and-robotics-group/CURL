function [points, weight, triangle_weight_coordinate,face_index,ray_tracing_time] = intersection_between_faces_lines_grid_mex(non_zero_data,faces,spherical_grid,recons_table_check)
    vertices = non_zero_data.points;
    face_planes = face_plane(vertices,faces);    
    points = zeros(size(spherical_grid,1),3);
    weight = zeros(size(spherical_grid,1),1);
    triangle_weight_coordinate = zeros(size(spherical_grid,1),2);
    face_index = zeros(size(spherical_grid,1),1);
    [ray_tracing_time,~,indexes] = ray_tracing_mex(vertices,faces,spherical_grid,recons_table_check.col);
%     [ray_tracing_time,fake_points,indexes] = ray_tracing_no_thres_mex(vertices,faces,spherical_grid,recons_table_check.col);
    disp(['C++ Sampling time: ' num2str(ray_tracing_time) 's']);
    tic
    [points(indexes>0,:), weight(indexes>0,:), triangle_weight_coordinate(indexes>0,:),face_index(indexes>0,:)]= intersection_between_faces_lines_parallel_CPU_with_area_mex(spherical_grid(indexes>0,1),spherical_grid(indexes>0,2),vertices,faces(indexes(indexes>0),:),face_planes(indexes(indexes>0),:));
    disp(['Matlab Calculation time ' num2str(toc) 's']);
    points(recons_table_check.col==1,:) = non_zero_data.allpoints; % add original points
    % used for draw fig4
    non_zero_data_r = vecnorm(non_zero_data.allpoints');
    count = 1;
    for i = 1:length(recons_table_check.col)
        if recons_table_check.col(i) == 1
            recons_table_check.col(i) = non_zero_data_r(count)>0;
            count = count+1;
        end
    end
end