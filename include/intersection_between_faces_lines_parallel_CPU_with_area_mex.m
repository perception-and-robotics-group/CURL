function [p_inter_result_mid, weight_mid, triangle_weight_coordinate, face_index_mid] = intersection_between_faces_lines_parallel_CPU_with_area_mex(azi,inc,vertices,faces,face_planes)
    direction = [cos(inc).*cos(azi) cos(inc).*sin(azi) sin(inc)];
    p_inter_result_mid = zeros(size(faces,1),3);
    weight_mid = zeros(size(faces,1),1);
    u_mid = zeros(size(faces,1),1);
    v_mid = zeros(size(faces,1),1);
    face_index_mid = zeros(size(faces,1),1);
    triangle_weight_coordinate = zeros(size(faces,1),2);
    
    cpu_direction = direction;
    cpu_p1 = vertices(faces(:,1),:);
    cpu_p2 = vertices(faces(:,2),:);
    cpu_p3 = vertices(faces(:,3),:);
    cpu_normal_vector = face_planes(:,1:3);
    cpu_D = face_planes(:,4);    
    cpu_detect_parallel = dot(cpu_normal_vector,cpu_direction,2);
    cpu_t = -cpu_D./cpu_detect_parallel;
    cpu_p_inter = cpu_t.*cpu_direction;
    cpu_cond_1 = dot(cross((cpu_p2-cpu_p1)',(cpu_p_inter-cpu_p1)'), cpu_normal_vector')';
    cpu_cond_2 = dot(cross((cpu_p3-cpu_p2)',(cpu_p_inter-cpu_p2)'), cpu_normal_vector')';
    cpu_cond_3 = dot(cross((cpu_p1-cpu_p3)',(cpu_p_inter-cpu_p3)'), cpu_normal_vector')';
    % detection
    cpu_index_1 = cpu_detect_parallel~=0;
    cpu_index_2 = cpu_cond_1>=0;
    cpu_index_3 = cpu_cond_2>=0;
    cpu_index_4 = cpu_cond_3>=0;
    cpu_index_5 = cpu_t > 0;
%     cpu_index_6 = cpu_area_detection < area_thres;
    cpu_final_index = find((cpu_index_1&cpu_index_2&cpu_index_3&cpu_index_4&cpu_index_5)>0);
    if ~isempty(cpu_final_index)
        for i = cpu_final_index'
            p_inter_result_mid(i,:) = cpu_p_inter(i,:);
            weight_mid(i,:) = 1./face_planes(i,5);
            x_vector = [cpu_p2(i,1)-cpu_p1(i,1) cpu_p3(i,1)-cpu_p1(i,1) cpu_p1(i,1)-cpu_p_inter(i,1)];
            y_vector = [cpu_p2(i,2)-cpu_p1(i,2) cpu_p3(i,2)-cpu_p1(i,2) cpu_p1(i,2)-cpu_p_inter(i,2)];
            cross_vector = cross(x_vector,y_vector);
            u_mid(i,:) = cross_vector(1)/cross_vector(3);
            v_mid(i,:) = cross_vector(2)/cross_vector(3);
            face_index_mid(i,:) = i;
        end
    end
    triangle_weight_coordinate = [u_mid v_mid];
%     if(~isempty(weight_mid))
%         [weight,index] = max(weight_mid);
%         p_inter_result = p_inter_result_mid(index,:);
%         triangle_weight_coordinate = [u_mid(index) v_mid(index)];
%         face_index = face_index_mid(index,:);
%     end
%     p_inter_result = double(p_inter_result);
%     weight = double(weight);
%     triangle_weight_coordinate = double(triangle_weight_coordinate);
%     face_index = double(face_index);
end

