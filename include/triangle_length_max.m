function [s, v] = triangle_length_max(vertices,faces)
    A = faces(:,1); B = faces(:,2); C = faces(:,3);
    a = sqrt(sum(((vertices(A,:)-vertices(B,:)).^(2))'))';
    b = sqrt(sum(((vertices(B,:)-vertices(C,:)).^(2))'))';
    c = sqrt(sum(((vertices(C,:)-vertices(A,:)).^(2))'))'; 
    s = [a b c];
    s = max(s,[],2);
    for i = 1:length(s)
        if(s(i) == a(i))
            v(i,1) = 1;
            v(i,2) = 2;
        elseif(s(i) == b(i))
            v(i,1) = 2;
            v(i,2) = 3;
        else
            v(i,1) = 1;
            v(i,2) = 3;
        end
    end
end

