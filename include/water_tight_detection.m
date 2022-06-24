function [is_water_tight, result] = water_tight_detection(faces)
    edges = [];
    count = 1;
    for i = 1:length(faces)
        k = 4;
        l = 4;
        m = 4;
        edges(count,1:2) = [faces(i,1) faces(i,2)];
        edges(count,3) = 0;
        edges(count+1,1:2) = [faces(i,2) faces(i,3)];
        edges(count+1,3) = 0;
        edges(count+2,1:2) = [faces(i,1) faces(i,3)];
        edges(count+2,3) = 0;
        
        for j = 1:length(faces)
            if((~isempty(find(faces(j,:)==edges(count,1), 1)))&&(~isempty(find(faces(j,:)==edges(count,2), 1))))
                edges(count,3) = edges(count,3)+1;
                edges(count,k) = j;
                k = k+1;
            end
            if((~isempty(find(faces(j,:)==edges(count+1,1), 1)))&&(~isempty(find(faces(j,:)==edges(count+1,2), 1))))
                edges(count+1,3) = edges(count+1,3)+1;
                edges(count+1,l) = j;
                l = l+1;
            end
            if((~isempty(find(faces(j,:)==edges(count+2,1), 1)))&&(~isempty(find(faces(j,:)==edges(count+2,2), 1))))
                edges(count+2,3) = edges(count+2,3)+1;
                edges(count+2,m) = j;
                m = m+1;
            end
        end
        count = count+3;
    end
    is_water_tight = isempty(find(edges(:,3)~=2, 1));
    result = edges;
end

