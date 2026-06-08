function tr = ukf_predict(tr, dt, sigma_acc, sigma_omega, omega_thresh, ukfP)
    n = ukfP.N;

    Q = diag([
        1e-4;
        1e-4;
        (sigma_acc * dt)^2;
        1e-4;
        (sigma_omega * dt)^2
    ]);

    lambdaA = ukfP.LAMBDA;
    scale   = n + lambdaA;

    if scale <= 0, scale = 1e-9; end

    Pa = (tr.P + tr.P')/2 + Q;
    Pa = Pa + 1e-9 * eye(n);

    sqrtPa = safeChol(scale * Pa);

    x  = tr.x;
    Xa = [x, x + sqrtPa, x - sqrtPa];

    Xprop = zeros(n, 2*n+1);
    for i = 1:(2*n+1)
        Xprop(:,i) = f_model(Xa(:,i), dt, omega_thresh);
    end

    x_pred = Xprop * ukfP.Wm';
    x_pred(4) = mod(x_pred(4) + pi, 2*pi) - pi;

    P_pred = zeros(n,n);
    for i = 1:(2*n+1)
        d = Xprop(:,i) - x_pred;
        d(4) = mod(d(4) + pi, 2*pi) - pi;
        P_pred = P_pred + ukfP.Wc(i) * (d*d');
    end

    tr.x = x_pred;
    tr.P = makeSPD(P_pred, 1e-9);
end
