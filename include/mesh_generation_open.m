function [index, faces, DT] = mesh_generation_open(vertices,mask,img_length)
    count = 1;
    for i = 1:length(mask.col)
        if (mask.col(i)==1)
            index(count,1) = ceil(i/img_length);
            index(count,2) = mod(i,img_length)*2*pi/img_length;
            count  = count +1;
        end
    end
    [projection_x, projection_y] = two_sph_2_cart(index(:,1),index(:,2));
    DT = delaunayTriangulation(projection_x,projection_y);
    % remove top faces
    count = 1;
    for i = 1:length(mask.col)
        if (mask.col(i)==1)
            index_inv(count,1) = 1/ceil(i/img_length);
            index_inv(count,2) = mod(i,img_length)*2*pi/img_length;
            count  = count +1;
        end
    end
    [projection_x_inv, projection_y_inv] = two_sph_2_cart(index_inv(:,1),index_inv(:,2));
    DT_inv = delaunayTriangulation(projection_x_inv,projection_y_inv);
    C = convexHull(DT_inv);
    C(length(C),:) = [];
    DT2 = delaunayTriangulation(DT_inv.Points(C,1),DT_inv.Points(C,2));
    count = 1;
    is_remove = 0;
    for i = 1:size(DT.ConnectivityList,1)
        for j = 1:size(DT2.ConnectivityList,1)
            if DT.ConnectivityList(i,:) == DT2.ConnectivityList(j,:)
                is_remove = 1;
            end
        end
        if (~is_remove)
            faces(count,:) = DT.ConnectivityList(i,:);
            count = count+1;
        end
        is_remove = 0;
    end
end

