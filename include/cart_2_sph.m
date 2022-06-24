function [azi, inc, r] = cart_2_sph(x,y,z)
    [azi,elev,r] = cart2sph(x,y,z);
    for i = 1:length(azi)
        if azi(i) < 0
            azi(i) = azi(i)+2*pi;
        end
    end
    inc = pi/2 - elev;
    clear x y z;
end