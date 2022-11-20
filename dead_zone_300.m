function out = dead_zone_300(r,u,R)
    pr = density_300(r,R);
    zr = max(u-r, min(0,u+r));
    out = pr*zr;
end