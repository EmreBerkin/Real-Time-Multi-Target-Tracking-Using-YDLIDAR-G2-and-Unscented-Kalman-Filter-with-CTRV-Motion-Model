function tr = ukf_update(tr, z, sigma_meas, ukfP)
    n      = ukfP.N;
    lambda = ukfP.LAMBDA;
    Wm     = ukfP.Wm;
    Wc     = ukfP.Wc;

    x_prior = tr.x;
    P_prior = makeSPD(tr.P, 1e-9);

    R = sigma_meas^2 * eye(2);
    scale = n + lambda;

    if scale <= 0, scale = 1e-9; end

    sqrtP = safeChol(scale * P_prior);
    X = [x_prior, x_prior + sqrtP, x_prior - sqrtP];

    Z = X(1:2, :);
    z_pred = Z * Wm';

    Pzz = R;
    Pxz = zeros(n, 2);

    for i = 1:(2*n+1)
        dz = Z(:,i) - z_pred;
        dx = X(:,i) - x_prior;
        dx(4) = mod(dx(4) + pi, 2*pi) - pi;

        Pzz = Pzz + Wc(i) * (dz * dz');
        Pxz = Pxz + Wc(i) * (dx * dz');
    end

    Pzz = makeSPD(Pzz, 1e-10);
    K   = Pxz / Pzz;
    innov = z - z_pred;

    x_post = x_prior + K * innov;
    x_post(4) = mod(x_post(4) + pi, 2*pi) - pi;

    P_post = P_prior - K * Pzz * K';
    P_post = makeSPD(P_post, 1e-9);

    tr.x = x_post;
    tr.P = P_post;

    tr.nis = innov' * (Pzz \ innov);

    dx_update = x_post - x_prior;
    dx_update(4) = mod(dx_update(4) + pi, 2*pi) - pi;
    tr.nees = dx_update' * pinv(P_post) * dx_update;

    if isnan(tr.nees) || isinf(tr.nees)
        tr.nees = NaN;
    end
end
