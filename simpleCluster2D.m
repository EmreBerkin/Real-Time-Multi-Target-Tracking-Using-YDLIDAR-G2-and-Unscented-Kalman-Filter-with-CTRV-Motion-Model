function labels = simpleCluster2D(pts, epsDist, minPts)
    N = size(pts,1);
    labels = zeros(N,1);
    visited = false(N,1);
    clusterId = 0;

    D = pdist2(pts, pts);

    for i = 1:N
        if visited(i), continue; end
        visited(i) = true;

        neighbors = find(D(i,:) <= epsDist);

        if numel(neighbors) < minPts
            labels(i) = -1;
            continue;
        end

        clusterId = clusterId + 1;
        labels(i) = clusterId;

        seedSet = neighbors(:);
        k = 1;

        while k <= numel(seedSet)
            j = seedSet(k);

            if ~visited(j)
                visited(j) = true;
                neighbors_j = find(D(j,:) <= epsDist);

                if numel(neighbors_j) >= minPts
                    seedSet = unique([seedSet; neighbors_j(:)]);
                end
            end

            if labels(j) == 0 || labels(j) == -1
                labels(j) = clusterId;
            end
            k = k + 1;
        end
    end
    labels = labels(:);
end
