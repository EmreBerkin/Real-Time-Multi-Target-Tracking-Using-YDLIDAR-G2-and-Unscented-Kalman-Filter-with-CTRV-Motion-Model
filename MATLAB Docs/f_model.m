function xn = f_model(x, dt, omega_thresh)
    px    = x(1);
    py    = x(2);
    v     = x(3);
    psi   = x(4);
    omega = x(5);

    if abs(omega) > omega_thresh
        px_new = px + (v/omega) * (sin(psi + omega*dt) - sin(psi));
        py_new = py + (v/omega) * (-cos(psi + omega*dt) + cos(psi));
        psi_new = psi + omega*dt;
    else
        px_new  = px + v*cos(psi)*dt;
        py_new  = py + v*sin(psi)*dt;
        psi_new = psi;
    end

    xn = [
        px_new;
        py_new;
        v;
        mod(psi_new + pi, 2*pi) - pi; % wrapToPiLocal inline
        omega
    ];
end
