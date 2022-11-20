function out = Pra_300(R,u)
    fun = @(x,u) dead_zone_300(x,u,R);
    out = integral(@(x) fun(x,u),0,R,'ArrayValued', true);
end