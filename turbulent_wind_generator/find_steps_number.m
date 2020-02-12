function N = find_steps_number(T,dt)
in = 10:16;
Ni = 2.^in;
[~,idx_N] = min(abs(bsxfun(@minus,Ni,round(T/dt))));
N = 2^in(idx_N);
end