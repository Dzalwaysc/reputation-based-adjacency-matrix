function out = p0_300(R)
    fun = @(x) density_300(x,R);
    out = integral(fun, 0, R, 'ArrayValued', true);
end