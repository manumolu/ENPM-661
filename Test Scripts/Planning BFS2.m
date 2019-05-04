% Dimensions of Workspace - 250X150
% 8 Connected Workspace

% INSERT START AND GOAL POINTS HERE--->
Start_Point= [0,0];
Goal_Point = [250,150];

%Setting the First Node to start Search
CurrentNode = Start_Point;

%Initializing Nodes,NodesInfo
Nodes = [];                  % Saves Unique visted Nodes
NodesInfo = [];              %Saves Information on given Node 
Nodes(:,:,1) = Start_Point;
%NodesInfo = Node#, Parent Node, Cost 
NodesInfo(:,:,1) = [1,0,0];
ParentNodeCount =1;
Cost=0;
N=1; %Number of Visted Nodes
CurrentNode = Nodes(:,:,ParentNodeCount);

%Plot Worskpace Figure

%Circular Obstacle
xc=180;
yc= 120;
t = 0:0.01:2*pi;        
x = 15*cos(t)+ xc;
y = 15*sin(t)+ yc;
drawnow
plot(x,y), hold on
fill(x,y,'b');

%Plot Polygons and Workspace Boundaries
xp1 = [ 55,105,105,55,55];
yp1 = [ 67.5,67.5,112.5,112.5,67.5] ;
drawnow
plot(xp1,yp1);
fill(xp1,yp1,'b');

xp2 = [145,168,188,165,158,120,145];
yp2 = [14,14,51,89,51,55,14];
drawnow
plot(xp2,yp2);
fill(xp2,yp2,'b');

xp3 = [0 ,250, 250, 0 ,0];
yp3 = [0 ,0, 150 ,150 ,0];
drawnow
plot(xp3,yp3);

alpha(0.3);

%Main Loop

while ~isequal(CurrentNode,Goal_Point)
CurrentNode = Nodes(:,:,ParentNodeCount);

%For Action Right
[StatusRight,NewNode] = ActionRight(CurrentNode);
 if StatusRight == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

%For Action DownRight
[StatusDownRight,NewNode] = ActionDownRight(CurrentNode);
 if StatusDownRight == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

%For Action Down
[StatusDown,NewNode] = ActionDown(CurrentNode);
 if StatusDown == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

%For Action DownLeft
[StatusDownLeft,NewNode] = ActionDownLeft(CurrentNode);
 if StatusDownLeft == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

%For Action Left
[StatusLeft,NewNode] = ActionLeft(CurrentNode);
 if StatusLeft == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

%For Action TopLeft
[StatusTopLeft,NewNode] = ActionTopLeft(CurrentNode);
 if StatusTopLeft == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

%For Action Top
[StatusTop,NewNode] = ActionTop(CurrentNode);
 if StatusTop == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

%For Action TopRight
[StatusTopRight,NewNode] = ActionTopRight(CurrentNode);
 if StatusTopRight == 1
    if ~(any(all(bsxfun(@eq,Nodes,NewNode))))
        N=N+1;
        Nodes(:,:,N) = NewNode;
        NodesInfo(:,:,N) = [N,ParentNodeCount,Cost];
    end
 end

ParentNodeCount = ParentNodeCount+1;
end

%Finding Optimal Path
ParentNodeCount = ParentNodeCount - 1;
Counter
i=2;
path(:,:,1) = CurrentNode;
Number = ParentNodeCount;
TrackInfo =[];
TrackInfo(:,:,1)= [1,Number];

while ~isequal(CurrentNode,Start_Point)    
P = NodesInfoSet(1,2,Number);
CurrentNode = Nodes(:,:,P);
path(:,:,i) = Nodes(:,:,P);
Nodes(:,:,Number);
TrackInfo(:,:,i)= [Number,P];
Number = NodesInfo(1,2,P);
i = i+1;
NodesSet(:,:,Number);
end

 for k = 1:i-1
 Pathx(k) = path(1,1,k);
 Pathy(k) = path(1,2,k);
 end

 drawnow
 plot(Pathx,Pathy,'linewidth',2);
 hold off




function [x,y] = CurrentLocation(CurrentNode)
x= CurrentNode(1);
y =CurrentNode(2);
end

function[StatusObstacle] = CheckObstacle(NewNode)
xc = NewNode(1);
yc = NewNode(2);

%Check if inside circle
xcentre=180;
ycentre = 120;
t = 0:0.01:2*pi;
xcircle = 15*cos(t)+ xcentre;
ycircle = 15*sin(t) + ycentre;

[inc,onc] = inpolygon(xc,yc,xcircle,ycircle);

% Check if inside two polygons 
%Rectangle
xp1 = [ 55,105,105,55,55];
yp1 = [ 67.5,67.5,112.5,112.5,67.5] ;

[inp1,onp1] = inpolygon(xc,yc,xp1,yp1);

%Polygon
xp2 = [145,168,188,165,158,120,145];
yp2 = [14,14,51,89,51,55,14];
[inp2,onp2] = inpolygon(xc,yc,xp2,yp2);

% Workspace
xp3 = [0 ,250, 250, 0 ,0];
yp3 = [0 ,0, 150 ,150 ,0];

[inw,onw] = inpolygon(xc,yc,xp3,yp3);

in = inc | inp1 | inp2 | (~inw);
on = onc | onp1 | onp2;
onobstacle_boundary = on;
inside_obstacle = in;
onworkspace_boundary = onw;
StatusObstacle = on|in|(~onw);
end
   
function[StatusRight,NewNode] = ActionRight(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x+1,y];
 [StatusObstacle] = CheckObstacle(NewNode);
 if StatusObstacle == 1  
 NewNode = CurrentNode;
 StatusRight =0;
 elseif StatusObstacle ==0 
 NewNode =[x+1,y];
 StatusRight =1;
 end
 end
 
function[StatusDownRight,NewNode] = ActionDownRight(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x+1,y-1];
 [StatusObstacle] = CheckObstacle(NewNode);
 if StatusObstacle == 1  
 NewNode = CurrentNode;
 StatusDownRight =0;
 elseif StatusObstacle ==0  
 NewNode =[x+1,y-1];
 StatusDownRight=1;
 end
 end
 
function[StatusDown,NewNode] = ActionDown(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x,y-1];
 [StatusObstacle] = CheckObstacle(NewNode);
 if StatusObstacle == 1  
 NewNode = CurrentNode;
 StatusDown =0;
 elseif StatusObstacle ==0 
 NewNode =[x,y-1];
 StatusDown=1;
 end
 end
 
function[StatusDownLeft,NewNode] = ActionDownLeft(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x-1,y-1];
 [StatusObstacle] = CheckObstacle(NewNode);
 if StatusObstacle == 1   
 NewNode = CurrentNode;
 StatusDownLeft =0;
 elseif StatusObstacle ==0 
     NewNode =[x-1,y-1];
     StatusDownLeft=1;
 end
 end
 
function[StatusLeft,NewNode] = ActionLeft(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x-1,y];
 [StatusObstacle] = CheckObstacle(NewNode);
if StatusObstacle == 1  
    NewNode = CurrentNode;
    StatusLeft =0;
elseif StatusObstacle ==0  
    NewNode =[x-1,y];
    Statusleft=1;
 end
 end
  
function[StatusTopLeft,NewNode] = ActionTopLeft(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x-1,y+1];
 [StatusObstacle] = CheckObstacle(NewNode);
 if StatusObstacle == 1  
    NewNode = CurrentNode;
    StatusTopLeft =0;
 elseif StatusObstacle ==0  
     NewNode =[x-1,y+1];
     StatusTopLeft=1;
 end
 end

function[StatusTop,NewNode] = ActionTop(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x,y+1];
 [StatusObstacle] = CheckObstacle(NewNode);
 if StatusObstacle == 1 
 NewNode = CurrentNode;
 StatusTop =0;
 elseif StatusObstacle ==0 
 NewNode =[x,y+1];
 StatusTop=1;
 end
 end

function[StatusTopRight,NewNode] = ActionTopRight(CurrentNode)
 [x,y] = CurrentLocation(CurrentNode);
 NewNode = [x+1,y+1];
 [StatusObstacle] = CheckObstacle(NewNode);
 if StatusObstacle == 1 
    NewNode = CurrentNode;
    StatusTopRight =0;
 elseif StatusObstacle ==0 
     NewNode =[x+1,y+1];
     StatusTopRight=1;
end
end
 
 



 