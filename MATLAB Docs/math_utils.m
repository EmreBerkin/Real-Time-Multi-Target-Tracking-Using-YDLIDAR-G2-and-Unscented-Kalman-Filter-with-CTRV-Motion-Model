function P = makeSPD(P, epsVal)
    P = real(P);
    P = (P + P') / 2;

    if any(isnan(P(:))) || any(isinf(P(:)))
        P = eye(size(P)) * epsVal;
        return;
    end

    [V,D] = eig(P);
    d = real(diag(D));
    d(d < epsVal) = epsVal;

    P = V * diag(d) * V';
    P = real((P + P') / 2);
    P = P + epsVal * eye(size(P));
end

function L = safeChol(P)
    P = real(P);
    P = (P + P') / 2;
    n = size(P,1);

    if any(isnan(P(:))) || any(isinf(P(:)))
        L = eye(n) * 1e-3;
        return;
    end

    jitter = 1e-12;
    for k = 1:14
        [L,flag] = chol(P + jitter * eye(n), 'lower');
        if flag == 0, return; end
        jitter = jitter * 10;
    end

    [V,D] = eig(P);
    d = real(diag(D));
    d(d < 1e-9) = 1e-9;

    Pfix = V * diag(d) * V';
    Pfix = real((Pfix + Pfix') / 2);
    Pfix = Pfix + 1e-8 * eye(n);

    [L,flag] = chol(Pfix, 'lower');
    if flag ~= 0
        L = eye(n) * sqrt(1e-6);
    end
end

function pts = covEllipse(P2, center, nSigma, nPts)
    theta  = linspace(0, 2*pi, nPts+1);
    circle = [cos(theta); sin(theta)];

    try
        L = safeChol(makeSPD(P2, 1e-9));
    catch
        L = eye(2) * 0.01;
    end
    pts = nSigma * L * circle + repmat(center, 1, nPts+1);
end
