function [ v_i ] = x2vi( x  )

phi=x(7);
theta=x(8);
psi=x(9);
v_b=x(4:6);

v_i = rotate_b2i(v_b, phi,theta,psi);

end

