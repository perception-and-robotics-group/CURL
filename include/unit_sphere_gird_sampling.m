function sph_verts_sampled = unit_sphere_gird_sampling(sph_verts,faces,face_index,triangle_weight_coordinate)
    sph_verts_sampled = zeros(size(face_index,1),3);
    for i = 1:size(face_index,1)
        if face_index(i)~=0
            p1 = sph_verts(faces(face_index(i),1),:);
            p2 = sph_verts(faces(face_index(i),2),:);
            p3 = sph_verts(faces(face_index(i),3),:);
            sph_verts_sampled(i,:) = triangle_weight_coordinate(i,1).*(p2-p1)+triangle_weight_coordinate(i,2).*(p3-p1)+p1;
        end
    end
    clear sph_verts faces face_index triangle_weight_coordinate;
end

