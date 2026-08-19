m = 40;
n = 60;
start = [10,10];
goal = [40,60];
directions = [1,0,1;0,1,1;-1,0,1;0,-1,1;...  % 右 上 左 下
              -1,1,sqrt(2);1,-1,sqrt(2);-1,-1,sqrt(2);1,1,sqrt(2)];% 左上 右下 左下 右上   
obstacle = [];

for i = 1:28
    obstacle = [obstacle;20,i];
    obstacle = [obstacle;40,41-i];
   if i <=10
        obstacle = [obstacle;25+i,20];
        obstacle = [obstacle;50+i,30];
        obstacle = [obstacle;40+i,20];
    end
end






