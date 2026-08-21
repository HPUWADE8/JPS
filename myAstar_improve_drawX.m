clc;
clear;
close;
% 改变了一下g和h的权重
tic;
m = 40;  %行
n = 60;  %列

start_node = [1,5]; % （2,3）表示该点左下角的格子
target_node = [59,39];
g_weight = 1;
h_weight = 1;
% m = 50;  % 行
% n = 70;  % 列
% 
% start_node = [5,25];     % 起点
% target_node = [50,8];   % 终点
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
% 画栅格
% for i =1:m
%    plot([0,n],[i,i],'k');  %k:黑色
%    hold on;
% end

% for i =1:n
%    plot([i,i],[0,m],'k');  %k:黑色
   
% end

axis equal  % x,y轴的刻度相等
xlim([0,n]);
ylim([0,m]);

% × 的半边长，与 myjps 的画法一致，且保持在单个栅格内。
hx = 0.4;

% 填充起点 绿色 根据四个点去填充 
drawX(start_node - 0.5, 'g', hx);
hold on;
% 填充终点 红色
drawX(target_node - 0.5, 'r', hx);

% 填充障碍物
drawX(obstacle - 0.5, 'k', hx);


%% 预处理 进行第一次搜索 挑f最小的节点，看其子节点的距离和路径情况，把本次挑出的节点设置为已探索

%初始化closelist (已探索区域)
closelist = start_node;
closelist_path = {start_node,start_node};  %元胞数组 左：某个已探索的节点  右：从起点到该已探索节点的最短路径
closelist_cost = 0;  %从起点到某个节点的最短路径，如start_node，到自己肯定是0


child_nodes = child_nodes_cal(start_node,m,n,obstacle,closelist); %搜索合规的子节点

%初始化openlist(探索候选区域)
openlist = child_nodes; % 待探索集 普通矩阵
for i = 1:size(openlist,1)  %循环size(openlist,1)次 元胞数组每一行记录了子节点本身和其父节点,最左边为待探索子节点，右边为从起点到该点的最短路径
    openlist_path{i,1} = openlist(i,:);
    openlist_path{i,2} = [start_node;openlist(i,:)];
end
% f=g+h
for i = 1:size(openlist,1)
    g = norm(start_node - openlist(i,1:2));   %sqrt(sum(abs(A.^2))) 这里算的是欧氏距离
    h = norm(target_node - openlist(i,1:2));
%h = abs(target_node(1) - openlist(i,1)) + abs(target_node(2) - openlist(i,2));
    f = g_weight*g+h_weight*h;
    openlist_cost(i,:) = [g,h,f];
end

%% 开始搜索 1.挑出f最小的节点
[~,min_idx] = min(openlist_cost(:,3));
parent_node = openlist(min_idx,:);
parent_node_record = parent_node;

%% 进入循环  开始第二次搜索
flag = 1;
cnt=0;
while flag
    child_nodes=child_nodes_cal(parent_node,m,n,obstacle,closelist);

    % 2.判断新加的这些子节点是否在openlist中，在则比较更新，不在则追加到openlist中
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag,openlist_idx] = ismember(child_node,openlist,"rows"); %in_flag为child——node中存在openlist的标志，以in_flag为基准
        %openlist——idx为child_node中存在的openlist元素在openlist的第几行
        %row表示把每一行看做一个整体
        g = openlist_cost(min_idx,1) +  norm(parent_node-child_node);      %通过新的父节点计算g值
     %  h = abs(child_node(1) - target_node(1)) + abs(child_node(2) - target_node(2));
        h = norm(target_node-child_node);
        f =g_weight*g+h_weight*h;

        if any(in_flag(:))
            if g<openlist_cost(openlist_idx,1)         %h肯定不变，判断新的父节点计算的g和原来的g比较，小的话说明通过当前父节点才是最短路径，需要更新路径
               openlist_cost(openlist_idx,1) = g;          
               openlist_cost(openlist_idx,3) = f;
               openlist_path{openlist_idx,2} = [openlist_path{min_idx,2};child_node];
%               disp(cnt);
            end
        else
            openlist(end+1,:) = child_node;
            openlist_cost(end+1,:) = [g,h,f];
            openlist_path{end+1,1} = child_node;
            openlist_path{end,2} = [openlist_path{min_idx,2};child_node];
        end
    end

% 3.此轮搜索完毕后，从openlist中移除代价最小的点到closelist
    closelist(end+1,:) = openlist(min_idx,:); %移除的是本次循环的父节点
    closelist_cost(end+1,1) = openlist_cost(min_idx,3);
    closelist_path(end+1,:) = openlist_path(min_idx,:);  %元胞数组赋值也需要用元胞承接,赋值[]即为空值，删除掉该元胞
    openlist(min_idx,:) = [];
    openlist_cost(min_idx,:) = [];  %删除该行
    openlist_path(min_idx,:) = [];  %删除该元胞

 
     % 初始化下一次搜索
 [~,min_idx] = min(openlist_cost(:,3));  %如果存在最小距离相同的点，则优先搜索索引靠前的
 parent_node = openlist(min_idx,:);
parent_node_record = [parent_node_record;parent_node];
 % 判断是否到达终点
 if isequal(parent_node,target_node)
     closelist(end+1,:) = openlist(min_idx,:);
    closelist_cost(end+1,1) = openlist_cost(min_idx,3);
    closelist_path(end+1,:) = openlist_path(min_idx,:); 
    flag = 0;
       drawX(openlist - 0.5, [0.5, 0.7, 1], hx);
  
    drawX(closelist(2:end,:) - 0.5, [0.7, 0.9, 1], hx);
    drawX(start_node - 0.5, 'g', hx);

% 填充终点 红色
drawX(target_node - 0.5, 'r', hx);
 end
 cnt = cnt+1;
end
disp(cnt);
elapsedTime = toc;
path_opt = closelist_path{end,2};
path_opt(:,1) = path_opt(:,1)-0.5;
path_opt(:,2) = path_opt(:,2)-0.5;
% scatter(path_opt(:,1),path_opt(:,2),'k');  %绘制散点
title_text = sprintf('路径代价%f, 探索格数%f, 耗时%.2f秒', closelist_cost(end,1), size(closelist,1), elapsedTime);
title(title_text);
plot(path_opt(:,1),path_opt(:,2),'k');


function child_nodes=child_nodes_cal(parent_node,m,n,obstacle,closelist)
    child_nodes = [];
    field = [1,1;n,1;n,m;1,m];
   for i= -1:1
       for j = -1:1
        if ~((i==0) && (j==0))
           child_node = [parent_node(1)+i,parent_node(2)+j];
           if inpolygon(child_node(1),child_node(2),field(:,1),field(:,2)) %判断是否在地图内内
                if ~ismember(child_node,obstacle,'rows')
                    child_nodes = [child_nodes;child_node];
                end
           end  
        end
     end
   end
   %排除已经在closelist里的点
  delete_idx = [];
  for i =1:size(child_nodes,1)
    if ismember(child_nodes(i,:),closelist,'rows')
        delete_idx(end+1,:) = i;
    end
  end
    child_nodes(delete_idx,:) = [];
end







