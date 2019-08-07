% importing this to use queues for breath first algo of solve_maze
import java.util.LinkedList
%Orientation Map
%               z=2
%
%               ^
%     z=3       y         z=1
%               x->
%
%              z=4


%initializing probability map
%global n
global bool_check
bool_check = 1;
n_prob=15*7*4;
global p
p=ones(7,15,4)/n_prob;
%read Sensor value will be stored in this
global SenVal
SenVal=zeros(1,8);
%Standard stuff copied from the code provided
global dim1
dim1= 32;
global dim2
dim2= 16;
global locationindex
locationindex = reshape(1:dim1*dim2,dim1,dim2);
global n
n = numel(locationindex);
%rand('twister',5489);
global bw
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white

%make blocks
global M
M = zeros(size(bw));
global Blocks
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1)
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end

%Figure showing map
figure; imagesc((bw+1).*M2); colormap(gray);
%Initializing world variable
global world
world=zeros(4,8,7,15);
global mask
mask=zeros(7,15);
%initialize_world initializes the world and mask variables (don't have to worry about the algorithm)
initialize_world;
heading=-1;
global s

%Remember to open com port before running this code

global target1 target2 
%for loading and unloading (this code doesn't have the functionality of going to unloading zone. it was to be implemented after concreting localization)
target1=[3,1]; %Just arbitrary values
target2=[1,1];
loading=1;

while(1==1)
    %disp("Running read_sensor");
    % using heading_bot along with dir in which bot is facing (estimate from probability map) to find global heading and then using it in the move function
    heading_bot=read_sensor;
    %disp("Running sense_u");
    p=sense_u(p,SenVal);
    %disp("Running solve_maze");
    %max(max(max(p)))
    %if max probability of a state is >0.8 means it is localized and now it should head to target1
    if max(max(max(p)))>0.8
        if loading==1
            %solve_maze gives the next heading wrt world orientation for the current position
            heading=solve_maze(target1);
        else
            heading=solve_maze(target2);
        end
        % if we're localized no need for knowing the bot's current heading (wrt itself)
        heading_bot=-1;
    end
    if heading_bot~=-1
        %Evaluating the most probable direction in which the bot is facing
        dir1=sum(sum(sum(p(:,:,1))));
        dir2=sum(sum(sum(p(:,:,2))));
        dir3=sum(sum(sum(p(:,:,3))));
        dir4=sum(sum(sum(p(:,:,4))));
        
        if dir1>max(max(dir2,dir3),dir4)
            dir=1;
        elseif dir2>max(max(dir1,dir3),dir4)
            dir=2;
        elseif dir3>max(max(dir1,dir2),dir4)
            dir=3;
        elseif dir4>max(max(dir1,dir2),dir3)
            dir=4;
        else
            dir=-1;
        end
        heading=mod(heading_bot+dir+2,4) + 1;
        if dir==-1
            heading=-1;
        end
        heading
        
    end
            
    %disp("Running move");
    p=move(p, heading);
    [bool,flag]=write_heading(s,heading);
    if flag==1
        % flag 1 implies we have reached loading zone
        loading=0;
        pause;
    end
end


% All the code below is for initialize_world, can skip that
function [bool]=isfeasible(x_travel,y_travel,dir)
    bool=-1;
    global dim1 dim2 M
    if dir==1
        if x_travel+5>dim1
            bool=0;
            return
        elseif sum(M(y_travel,x_travel:x_travel+5))>0
            bool=0;
            return
        else
            bool=1;
        end
    elseif dir==2
        if y_travel-5<1
            bool=0;
            return
        elseif sum(M(y_travel-5:y_travel,x_travel))>0
            bool=0;
            return
        else
            bool=1;
        end
    elseif dir==3
        if x_travel-5<1
            bool=0;
            return
        elseif sum(M(y_travel,x_travel-5:x_travel))>0
            bool=0;
            return
        else
            bool=1;
        end
    elseif dir==4
        if y_travel+5>dim2
            bool=0;
            return
        elseif sum(M(y_travel:y_travel+5,x_travel))>0
            bool=0;
            return
        else
            bool=1;
        end
    end
end

function [sense1]=transform90(sense)
    sense1=sense;
    sense1(1:end-2)=sense(3:end);
    sense1(end-1:end)=sense(1:2);
end
function [bool]=isobstacle(x_temp,y_temp)
    global M
    sum1=sum(M(y_temp,x_temp-1:x_temp))+sum(M(y_temp-1:y_temp,x_temp));
    if (sum1>0)
        bool=0;
    else
        bool=1;
    end
end
function [] = initialize_world()
    global world mask
    x=1;
    x_map=3;y_map=3;
    for x_temp=x_map:2:31
        y=1;
        for y_temp=y_map:2:15
            mask(y,x)=isobstacle(x_temp,y_temp);
            sense_temp=zeros(1,8);
            x_travel=x_temp-2;
            y_travel=y_temp-2;
            sense_temp(7)=isfeasible(x_travel,y_travel,3);
            sense_temp(6)=isfeasible(x_travel,y_travel,2);
            y_travel=y_travel+2;
            sense_temp(8)=isfeasible(x_travel,y_travel,3);
            sense_temp(1)=isfeasible(x_travel,y_travel,4);
            x_travel=x_travel+2;
            sense_temp(3)=isfeasible(x_travel,y_travel,1);
            sense_temp(2)=isfeasible(x_travel,y_travel,4);
            y_travel=y_travel-2;
            sense_temp(4)=isfeasible(x_travel,y_travel,1);
            sense_temp(5)=isfeasible(x_travel,y_travel,2);
            world(1,:,y,x)=sense_temp;
            world(2,:,y,x)=transform90(world(1,:,y,x));
            world(3,:,y,x)=transform90(world(2,:,y,x));
            world(4,:,y,x)=transform90(world(3,:,y,x));
            y=y+1;
        end
        x=x+1;
    end
end
