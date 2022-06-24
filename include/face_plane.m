function face_planes = face_plane(vertices,faces)
    face_planes = zeros(size(faces,1),4);
    for i = 1:size(faces,1)
        p1 = vertices(faces(i,1),:);
        p2 = vertices(faces(i,2),:);
        p3 = vertices(faces(i,3),:);
        cross_product = cross((p2-p1),(p3-p1));
        area = norm(cross_product)/2;
        norm_vector = cross_product./(area*2);
        D = -norm_vector*p1';
        face_planes(i,1:3) = norm_vector(1:3);
        face_planes(i,4) = D;
        face_planes(i,5) = area;
    end
end

