close all;
clear ;
clc;
m = 40; %y
n = 60; %x
tic;
start = [1,5];
goal = [59,39];
directions = [1,0,1;0,1,1;-1,0,1;0,-1,1;...  % 右 上 左 下
              -1,1,sqrt(2);1,-1,sqrt(2);-1,-1,sqrt(2);1,1,sqrt(2)];% 左上 右下 左下 右上
obstacle = [];

for i = 1:28
    obstacle = [obstacle;20,i+1];
    obstacle = [obstacle;40,41-i];
   if i <=10
        obstacle = [obstacle;25+i,20];
        obstacle = [obstacle;50+i,30];
        obstacle = [obstacle;40+i,20];
    end
end


%% 可视化：标记边界、障碍、起点和终点
figure;
hold on;
axis equal;
xlim([-1, n+2]);
ylim([-1, m+2]);
set(gca, 'XTick', 0:20:n, 'YTick', 0:20:m);   % 笛卡尔坐标系，每隔 20 显示一条网格线
grid on;
hx = 0.4;   % × 的半边长：整边长 0.8 < 1，保证大小不超过 1×1
% 边界点（网格四周一圈），[行, 列]
boundary = [];
for i = 1:n
    boundary = [boundary; i, 0; i, m+1];    % 上、下边
end
for i = 1:m
    boundary = [boundary; 0, i; n+1, i];    % 左、右边
end
boundary = [boundary; 0, 0; 0, m+1; n+1, 0; n+1, m+1];
% 障碍 + 边界：黑色 ×
obstacle = [obstacle; boundary];
drawX(obstacle, 'k', hx);
% 起点：绿色 ×
drawX(start, 'g', hx);
% 终点：红色 ×
drawX(goal, 'r', hx);
hold off;

openlist = {};
closelist = {};

openlist{1,1} = start;
g_temp = 0;  h_temp = norm(start-goal) ; f_temp = g_temp+h_temp;
openlist{1,2} = [g_temp,h_temp,f_temp]; % g,h,f
openlist{1,3} = start; % 父节点坐标
openlist{1,4} = start; % 路径
flag = 0; % 标记是否找到路径
cnt = 0;
while flag==0
    cnt = cnt + 1;
    openlist_node = vertcat(openlist{:,1});
    openlist_cost = vertcat(openlist{:,2});
    openlist_cost_f = openlist_cost(:,3);
    [~,min_idx] = min(openlist_cost_f);
    parent_node = openlist_node(min_idx,:);
    parent_node_record = parent_node; % 记录每次探索的节点


    closelist{end+1,1} = parent_node;
    closelist{end,2} = openlist_cost(min_idx,:);
    closelist{end,3} = openlist{min_idx,3};
    closelist{end,4} = openlist{min_idx,4};
    if isequal(parent_node,goal)
        flag = 1;
        break;
    end
    successors_inthisloop = [];
    for i = 1:size(directions,1)
        direction = directions(i,1:2);
        successors = findsuccessor(parent_node,goal,m,n,obstacle,vertcat(closelist{:,1}),direction);
        successors_inthisloop = [successors_inthisloop; successors];
    end
    new_successors = setdiff(successors_inthisloop,vertcat(openlist{:,1}),'rows'); % 去除已在openlist中的后继结点
    %  if isequal(parent_node,goal)

    
    openlist(min_idx,:) = []; % 从openlist中删除当前节点
    drawX(new_successors, 'cyan', hx); 
    hold on;

    % 计算新后继结点的g、h、f值，并加入openlist
    for i = 1:size(new_successors,1)
        new_node = new_successors(i,:);
        g_temp = closelist{end,2}(1) + norm(parent_node - new_node); % g值为父节点的g值加上当前节点到父节点的距离
        h_temp = norm(new_node - goal);
        f_temp = g_temp + h_temp; % f值为g值加上h值
        openlist{end+1,1} = new_node;
        openlist{end,2} = [g_temp,h_temp,f_temp];
        openlist{end,3} = parent_node; % 父节点坐标
        openlist{end,4} = [closelist{end,4};new_node];
        % new_node = new_successors(i,:);
        % g_temp = closelist{end,2}(1) + norm(new_node-parent_node); % g值为父节点的g值加上当前节点到父节点的距离
        % h_temp = norm(new_node-goal); % h值为当前节点到目标节点的距离
        % f_temp = g_temp + h_temp; % f值为g值加上h值
        % openlist{end+1,1} = new_node;
        % openlist{end,2} = [g_temp,h_temp,f_temp];
        % openlist{end,3} = parent_node; % 父节点坐标
        % openlist{end,4} = [closelist{end,4};new_node]; % 路径
    end


end
elapsedTime = toc;

path = closelist{end,4};
plot(path(:,1),path(:,2),'r-','LineWidth',2);
pathcost = closelist{end,2}(1);
title(sprintf('路径长度为：%.2f,耗时：%.2f秒', pathcost, elapsedTime));