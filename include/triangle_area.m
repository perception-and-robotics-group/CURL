function obj_area = triangle_area(vertices,faces)
    A = faces(:,1); B = faces(:,2); C = faces(:,3);
    a = sqrt(sum(((vertices(A,:)-vertices(B,:)).^(2))'))';
    b = sqrt(sum(((vertices(B,:)-vertices(C,:)).^(2))'))';
    c = sqrt(sum(((vertices(C,:)-vertices(A,:)).^(2))'))';
    s = (a+b+c)/2;
    obj_area = sqrt(s.*(s-a).*(s-b).*(s-c)); % 海伦公式
    % obj_area = obj_area/sum(obj_area);
end

