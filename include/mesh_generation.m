function [index, faces, DT] = mesh_generation(vertices,mask,img_length)
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
    % add the bottom faces
    C = convexHull(DT);
    C(length(C),:) = [];
    DT2 = delaunayTriangulation(DT.Points(C,1),DT.Points(C,2));
    connectivity_list_compensate = [C(DT2.ConnectivityList(:,1)) C(DT2.ConnectivityList(:,3)) C(DT2.ConnectivityList(:,2))];%变成顺时针的面
    faces = [DT.ConnectivityList; connectivity_list_compensate];
%     faces = DT.ConnectivityList;
    % save procedure figure
    is_figure = false;
    if is_figure
        fig1 = figure('Name','Projection Points');
        plot(projection_x,projection_y,'.','MarkerSize',0.01);
        daspect([1 1 1])    
        ax = gca;
        ax.YAxis.Visible = "off";
        ax.XAxisLocation = "origin";
        ax.Box = "off";
        axp = get(gca,'Position');
        xs=axp(1);
        xe=axp(1)+axp(3);
        ys=axp(2);
        ye=axp(2)+axp(4);
        annotation('arrow', [xs+0.1*xe xe-0.1*xe],[(ys+ye)/2 (ys+ye)/2]);
        annotation('arrow', [(xs+xe)/2 xe*14/16*0.95],[(ys+ye)/2 ye*14/16*0.95]);
        ht = text(1.8,1.2,'\phi','FontSize',20,'FontWeight','bold');
        ax.FontSize = 12;
        xlabel('n','FontWeight','bold','FontSize',20,'Position',[14 -2.5]);
        ax.TickLength = [0 0];  
        set(gca,'Layer','top');
        uistack(ht,'top');
        print(fig1,'./experiments_figure/mesh_generation_a','-dpdf','-r600');
        hold on
        triplot(DT);
        set(gca,'Layer','top');
        uistack(ht,'top');
        print(fig1,'./experiments_figure/mesh_generation_b','-dpdf','-r600');
        fig2 = figure('Name','Convex');
        plot(DT.Points(C,1),DT.Points(C,2),'.','MarkerSize',0.01);
        daspect([1 1 1])    
        ax = gca;
        ax.YAxis.Visible = "off";
        ax.XAxisLocation = "origin";
        ax.Box = "off";
        axp = get(gca,'Position');
        xs=axp(1);
        xe=axp(1)+axp(3);
        ys=axp(2);
        ye=axp(2)+axp(4);
        annotation('arrow', [xs+0.1*xe xe-0.1*xe],[(ys+ye)/2 (ys+ye)/2]);
        annotation('arrow', [(xs+xe)/2 xe*14/16*0.95],[(ys+ye)/2 ye*14/16*0.95]);
        ht = text(1.8,1.2,'\phi','FontSize',20,'FontWeight','bold');
        ax.FontSize = 12;
        xlabel('n','FontWeight','bold','FontSize',20,'Position',[14 -2.5]);
        ax.TickLength = [0 0];    
        set(gca,'Layer','top');
        uistack(ht,'top');
        print(fig2,'./experiments_figure/mesh_generation_c','-dpdf','-r600');
        hold on
        triplot(DT2);
        set(gca,'Layer','top');
        uistack(ht,'top');
        print(fig2,'./experiments_figure/mesh_generation_d','-dpdf','-r600');
    end
    clear vertices mask img_length;
end

