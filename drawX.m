function drawX(pts, color, h)
    for k = 1:size(pts, 1)
        c = pts(k, 1);   % x
        r = pts(k, 2);   % y
        line([c-h, c+h], [r-h, r+h], 'Color', color, 'LineWidth', 1.5);
        line([c-h, c+h], [r+h, r-h], 'Color', color, 'LineWidth', 1.5);
    end
end
