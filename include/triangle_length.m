function s = triangle_length(vertices,faces)
    A = faces(:,1); B = faces(:,2); C = faces(:,3);
    a = sqrt(sum(((vertices(A,:)-vertices(B,:)).^(2))'))';
    b = sqrt(sum(((vertices(B,:)-vertices(C,:)).^(2))'))';
    c = sqrt(sum(((vertices(C,:)-vertices(A,:)).^(2))'))';
    s = a+b+c;
end

