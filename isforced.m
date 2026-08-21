function ishave = isforced(current_node,obstacle,direction);
    x_dir = direction(1);
    y_dir = direction(2);
    maybeobstacle = [];
    if x_dir == 0 && y_dir ~= 0 % 上下方向
        maybeobstacle = [current_node(1)-1,current_node(2);current_node(1)+1,current_node(2)];
    elseif x_dir ~= 0 && y_dir == 0 % 左右方向
        maybeobstacle = [current_node(1),current_node(2)-1;current_node(1),current_node(2)+1];
    elseif x_dir ~= 0 && y_dir ~= 0 % 对角线方向
        maybeobstacle = [current_node(1)-x_dir,current_node(2);current_node(1),current_node(2)-y_dir];
    end
    ishave = false;
    cnt = 0;
    for i = 1:size(maybeobstacle,1)
        if ismember(maybeobstacle(i,:),obstacle,'rows')
            cnt = cnt+1;
        end
    end
    if cnt > 0  %% 还需要解决两堵墙合一起的问题
        ishave = true;
    end
end