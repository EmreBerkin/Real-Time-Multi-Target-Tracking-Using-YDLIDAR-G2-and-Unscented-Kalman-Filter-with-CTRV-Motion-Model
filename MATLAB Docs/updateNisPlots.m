function updateNisPlots(figNIS, tracks, winLen, chiLo, chiHi)
    % NIS panelini güncelleyerek filtrenin istatistiksel uyumunu çizdirir.
    if ~isvalid(figNIS), return; end
    
    % Ana script'teki global handle dizilerine (nisAxes, nisLines, nisWarns) ulaşım yöntemi:
    % Bu handle'ları nesne üzerinden çekmek temiz bir yöntemdir.
    nisAxes  = findobj(figNIS, 'Type', 'axes');
    % Not: subplot sırası ters gelebileceğinden doğrudan slot indexi track içinden yönetilir.
    
    for ti = 1:numel(tracks)
        tr = tracks{ti}; 
        if ~tr.confirmed || isempty(tr.nisHistory), continue; end
        
        slot = tr.plotSlot; 
        nisHist = tr.nisHistory; 
        Nnis = numel(nisHist);
        idxWin = max(1, Nnis - winLen + 1):Nnis; 
        yData = nisHist(idxWin); 
        xData = 1:numel(yData);

        % İlgili subplot'un elemanlarını bulup güncelleme:
        hAx = nisAxes(7 - slot); % MATLAB figür altındaki eksenleri ters sırada tutabilir
        hLine = findobj(hAx, 'Type', 'line', '-and', 'Marker', 'o');
        hWarn = findobj(hAx, 'Type', 'line', '-and', 'Marker', 'x');

        if ~isempty(hLine), set(hLine, 'XData', xData, 'YData', yData, 'Color', tr.color); end
        
        outMask = (yData < chiLo) | (yData > chiHi);
        if any(outMask)
            if ~isempty(hWarn), set(hWarn, 'XData', xData(outMask), 'YData', yData(outMask), 'Color', [1 0.9 0.1]); end
        else
            if ~isempty(hWarn), set(hWarn, 'XData', nan, 'YData', nan); end
        end

        pctOK = 100 * mean(~outMask);
        if tr.missed > 0
            statusStr = sprintf('T%d NIS | miss=%d | OK %.0f%%', tr.id, tr.missed, pctOK); 
        else
            statusStr = sprintf('T%d NIS | ACTIVE | OK %.0f%%', tr.id, pctOK); 
        end
        title(hAx, statusStr, 'Color', tr.color); 
        xlim(hAx, [0 winLen]); 
        ylim(hAx, [0 max(chiHi * 2.5, 15)]);
    end
end
