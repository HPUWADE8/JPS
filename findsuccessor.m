% input: current_node, goal_node, m, n, obstacle, closelist_node, direction
% output: successors 单个方向的后继结点

function successors = findsuccessor(current_node,goal_node,m,n,obstacle,closelist_node,direction)
    successors = [];
    current_node = current_node + direction(1:2);
    if ismember(current_node,obstacle,'rows') || ismember(current_node,closelist_node,'rows')
        return;
    end

    if current_node(1) < 0 || current_node(1) > n+1 || current_node(2) < 0 || current_node(2) > m+1
        return;
    end

    if isequal(current_node,goal_node)
        successors = [successors;current_node];
        return;
    end   
    
    ishave= isforced(current_node,obstacle,direction);
    if ishave
        successors = [successors;current_node];
        return;
    end
    
    if direction(1) ~= 0 && direction(2) ~= 0 % 对角线方向
        stright_directions = [direction(1),0,1;0,direction(2),1];
        for i = 1:size(stright_directions,1)
            stright_new_node = current_node + stright_directions(i,1:2);
            if ~isempty(findsuccessor(stright_new_node,goal_node,m,n,obstacle,closelist_node,stright_directions(i,:)))
                successors = [successors;current_node];
                return;
            end
        end
    end

    successors = findsuccessor(current_node, goal_node, m, n, obstacle, closelist_node, direction);
end