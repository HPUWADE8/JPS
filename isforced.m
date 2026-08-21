function ishave = isforced(current_node,obstacle,direction);
    x_dir = direction(1);
    y_dir = direction(2);
    maybeobstacle = [];
    ishave = false;
    if x_dir == 0 && y_dir ~= 0 % 上下方向
        maybeobstacle = [current_node(1)-1,current_node(2);current_node(1)+1,current_node(2)];  % 左右的障碍
        maybeforcedneibor = [current_node(1)-1,current_node(2)+y_dir;current_node(1)+1,current_node(2)+y_dir];
        for i = 1:size(maybeobstacle,1)
            if ismember(maybeobstacle(i,:),obstacle,'rows') && ~ismember(maybeforcedneibor(i,:),obstacle,'rows')
                ishave = true;  
            end
        end
    elseif x_dir ~= 0 && y_dir == 0 % 左右方向
        maybeobstacle = [current_node(1),current_node(2)-1;current_node(1),current_node(2)+1];
        maybeforcedneibor = [current_node(1)+x_dir,current_node(2)-1;current_node(1)+x_dir,current_node(2)+1];
        for i = 1:size(maybeobstacle,1)
            if ismember(maybeobstacle(i,:),obstacle,'rows') && ~ismember(maybeforcedneibor(i,:),obstacle,'rows')
                ishave = true;  
            end
        end
    elseif x_dir ~= 0 && y_dir ~= 0 % 对角线方向
        maybeobstacle = [current_node(1)-x_dir,current_node(2);current_node(1),current_node(2)-y_dir];
        maybeforcedneibor = [current_node(1)-x_dir,current_node(2)+y_dir;current_node(1)+x_dir,current_node(2)-y_dir];
        for i = 1:size(maybeobstacle,1)
            if ismember(maybeobstacle(i,:),obstacle,'rows') && ~ismember(maybeforcedneibor(i,:),obstacle,'rows')
                ishave = true;  
            end
        end
    end

end