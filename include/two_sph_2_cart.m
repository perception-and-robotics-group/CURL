function [x,y] = two_sph_2_cart(r,theta)
    x = r.*cos(theta);
    y = r.*sin(theta);
end

