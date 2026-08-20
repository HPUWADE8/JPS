clc;
clear all;
close all;

openlist = {};
openlist{1,1} = [1,2];
openlist{1,2} = [0,0,0]; % g,h,f
openlist{1,3} = [1,2]; % 父节点坐标
openlist{2,1} = [1,3];
openlist{2,2} = [0,0,0]; % g,h,f
openlist{2,3} = [1,2]; % 父节点坐标