B.e0  = 12.69; % [V]
B.e1  = -3.14; % [V]
B.e2  = 1.58;  % [V]
B.A   = 1.53;  % [V]
B.B   = 29.89; % [Ah^(-1)]
B.V0  = 12.59; % [V]
B.Q0  = 0.034; % [Ah]
B.Qf  = 1.20;  % [Ah]
B.R   = 0.061; % [Ohm]
B.tau = 1.95;  % [s]

B.p1  = 1.31e-7;
B.p2  = 4.03e-15;
B.p3  = -1.22e-23;
B.p4  = 1.65e-31;

Pboard = 0;

% Pboard = ;


nominal_V = @(Q)   B.e0 + B.e1*(Q/B.Qf) + B.e2*(Q/B.Qf)^2; %[V]
power   = @(rpm)   0.73*(B.p1*rpm^2 + B.p2*rpm^4 + B.p3*rpm^6 + B.p4*rpm^8); %[W]

Q       = B.Q0;
V       = B.V0;
i       = zeros(5,1);

w1 = 800*30/pi;  %rad/s to rpm
w2 = 800*30/pi;
w3 = 800*30/pi;
w4 = 800*30/pi;

window_size = 5; 
b = (1/window_size)*ones(1,window_size);
a = 1;


dt = 0.1;
t = 0;
figure;

while Q < B.Qf
    % Sum board and motors power
    t           = t + dt;
    Pow         = Pboard + power(w1) + power(w2) + power(w3) + power(w4);
    i           = circshift(i, -1);
    i(end)      = Pow/V;
    signal_filt = filter(b,a,i);
    i_filt      = signal_filt(end);

    Q       = Q + i_filt*dt/3600; %[Ah]
    Vf      = B.A*exp(-B.B*(B.Qf-B.Q0-Q));
    V_nom   = nominal_V(Q);
    V       = V_nom - B.R*i_filt - Vf;
    
    plot(t,V,'bo', t,Q,'ro');
    hold on;

end

