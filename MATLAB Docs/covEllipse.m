function ellXY = covEllipse(P_pos, center, nScale, numPoints)
    % 2D Durum Kovaryans Matrisinden Çizim için Elips Noktaları Üretir
    if nargin < 4, numPoints = 32; end
    
    th = linspace(0, 2*pi, numPoints);
    unitCircle = [cos(th); sin(th)];
    
    % Kovaryans matrisinin karekökünü (Cholesky) alarak elipsi ölçekleriz
    try
        L = chol(P_pos, 'lower');
        ellXY = center + nScale * (L * unitCircle);
    catch
        % Matris pozitif tanımlı değilse eig kullanılarak kurtarma yapılır
        [V, D] = eig(P_pos);
        ellXY = center + nScale * (V * sqrt(max(D, 0)) * unitCircle);
    end
end
