function clusterList = extractClusters(x_frame, y_frame, maxDetRange, minPoints, epsM, params)
    % Yakın mesafedeki noktaları bulup kümeleme (DBSCAN) algoritmasından geçirir.
    % params: [MIN_W, MIN_H, MAX_W, MAX_H] limitlerini içeren bir struct olmalıdır.
    
    r_frame = sqrt(x_frame.^2 + y_frame.^2);
    detIdx = r_frame <= maxDetRange;
    pts = [x_frame(detIdx), y_frame(detIdx)];
    pts = pts(~any(isnan(pts) | isinf(pts), 2), :);

    clusterList = {};
    if size(pts, 1) < minPoints
        return;
    end
    
    if exist('dbscan', 'file') == 2
        lbls = dbscan(pts, epsM, minPoints);
    else
        lbls = simpleCluster2D(pts, epsM, minPoints);
    end
    
    lbls = lbls(:); 
    Npts = min(length(lbls), size(pts, 1));
    lbls = lbls(1:Npts); 
    pts = pts(1:Npts, :);
    uLbls = unique(lbls); 
    uLbls(uLbls <= 0) = [];

    for c = uLbls(:)'
        mask = (lbls == c); 
        cPts = pts(mask, :);
        if size(cPts, 1) < minPoints, continue; end
        
        xmin = min(cPts(:, 1)); xmax = max(cPts(:, 1)); 
        ymin = min(cPts(:, 2)); ymax = max(cPts(:, 2));
        w = xmax - xmin; 
        h = ymax - ymin;
        
        % Geometrik boyut filtrelemesi
        if w < params.MIN_TARGET_WIDTH_M || h < params.MIN_TARGET_HEIGHT_M || ...
           w > params.MAX_TARGET_WIDTH_M || h > params.MAX_TARGET_HEIGHT_M
            continue;
        end

        S.pts = cPts; 
        S.center = mean(cPts, 1); 
        S.bbox = [xmin, ymin, w, h]; 
        S.w = w; 
        S.h = h;
        clusterList{end+1} = S; %#ok<AGROW>
    end
end
