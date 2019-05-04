% Robotics    : Planning for Autonomous Robots 
% Project - 5 : Motion Planning and Control in Urban Parking Spaces
% Author      :(1) Gireesh Suresh  - gireesh@umd.edu
%              (2) Manohar Anumolu - manoharanumolu.96@gmail.com
% School      : University of Maryland, College Park


%% Clean Slate

close all;
clear all;
clc;

%% Switch to the current directory of mfile.

if(~isdeployed)
  cd(fileparts(which(mfilename)));
end

%%  Global Planner using A* Algorithm

%Initial Conditions //Specify START AND GOAL POINTS HERE ---->
tic
Res =0.1; %----> Resolution
Start_Point =[1350,900]*Res; %Multiplying for different resolutions
Goal_Point =[140,200]*Res;

Nodes= []; % All Different Visited Nodes
NodesInfo = []; 
N=1;
CurrentNode = Start_Point;
Nodes(:,:,1) = CurrentNode;
ctc = 0; %Initializing Cost
ParentNodeCount=1;
CurrentNodeCount=size(CurrentNode,3);
site = Position(CurrentNode);
L_Cost =1;     %Linear Traversal Cost           
D_Cost =1.4;   %Diagonal Traversal Cost
H_Cost = norm(Goal_Point - CurrentNode);
F_Cost = ctc + H_Cost;
NodesInfo(:,:,1) = [1,1,ctc, F_Cost,site];

%Generates Tentative Map of the Parking Lot in 2D

%Plotting Scene
% Table1
XSq = [[-600,-600,-650,-650,-600]+750]*Res;
YSq = [[-500,-350,-350,-500,-500]+500]*Res;
drawnow 
plot(XSq,YSq);
hold on
fill(XSq,YSq,'b');

%Table2
XSq1 = [[-400,-400,-450,-450,-400]+750]*Res;
YSq1 = [[-500,-350,-350,-500,-500]+500]*Res ;
drawnow
plot(XSq1,YSq1);
fill(XSq1,YSq1,'b');

%Table3
XSq2 = [[-200,-200,-250,-250,-200]+750]*Res;
YSq2 = [[-500,-350,-350,-500,-500]+500]*Res ;
drawnow
plot(XSq2,YSq2);
fill(XSq2,YSq2,'b');

%Table4
XSq3 = [[0 0 50 50 0]+750]*Res;
YSq3 = [[-500,-350,-350,-500,-500]+500]*Res ;
drawnow
plot(XSq3,YSq3);
fill(XSq3,YSq3,'b');

XSq4 = [[-600,-600,-650,-650,-600]+750]*Res;
YSq4 = [[-500,-350,-350,-500,-500]+500+500]*Res;
drawnow
plot(XSq4,YSq4);
fill(XSq4,YSq4,'b');


XSq5 = [[-400,-400,-450,-450,-400]+750]*Res;
YSq5 = [[-500,-350,-350,-500,-500]+500+500]*Res ;
drawnow
plot(XSq5,YSq5);
fill(XSq5,YSq5,'b');

XSq6 = [[-200,-200,-250,-250,-200]+750]*Res;
YSq6 = [[-500,-350,-350,-500,-500]+500+500]*Res ;
drawnow
plot(XSq6,YSq6);
fill(XSq6,YSq6,'b');

XSq7 = [[0 0 50 50 0]+750]*Res;
YSq7 = [[-500,-350,-350,-500,-500]+500+500]*Res ;
drawnow
plot(XSq7,YSq7);
fill(XSq7,YSq7,'b');

xp3 = [[-750,-750 750,750,-750]+750]*Res;
yp3 = [[-500,500,500,-500,-500]+500]*Res;
drawnow
plot(xp3,yp3);
alpha(0.3);


while ~isequal(CurrentNode,Goal_Point)
    CurrentNode = Nodes(:,:,ParentNodeCount);
    %Action Right
   [StatusRight,NewNode] = ActionRight(CurrentNode);  
     if(StatusRight ==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = L_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = L_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp, min_C,min_F,site_new];
     end
     end
     end
    
    %Action Top Right
   [StatusTopRight,NewNode] = ActionTopRight(CurrentNode);  
     if(StatusTopRight==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = D_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = D_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp, min_C,min_F,site_new];
     end
     end
     end           
     
    %Action Top
    [StatusTop,NewNode] = ActionTop(CurrentNode);  
     if(StatusTop==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = L_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = L_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp, min_C,min_F,site_new];
     end
     end
     end        
     
    %Action Topleft
    [StatusTopLeft,NewNode] = ActionTopLeft(CurrentNode);  
     if(StatusTopLeft==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = D_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = D_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp, min_C,min_F,site_new];
     end
     end
     end
     
     %Action Left
    [StatusLeft,NewNode] = ActionLeft(CurrentNode);  
     if(StatusLeft==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = L_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = L_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp, min_C,min_F,site_new];
     end
     end
     end     
     
     %Action DownLeft
    [StatusDownLeft,NewNode] = ActionDownLeft(CurrentNode);  
     if(StatusDownLeft==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = D_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = D_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp, min_C,min_F,site_new];
     end
     end
     end 
     
     %Action Down
    [StatusDown,NewNode] = ActionDown(CurrentNode);  
     if(StatusDown==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = L_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = L_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp, min_C,min_F,site_new];
     end
     end
     end 
     
     %Action DownRight
    [StatusDownRight,NewNode] = ActionDownRight(CurrentNode);  
     if(StatusDownRight==1)
   [In_Obs,On_Obs] = Check_Obstacle(NewNode);
     if~(In_Obs||On_Obs)
         site_new = Position(NewNode);
     if(~(any(site_new == NodesInfo(1,5,:))))
         N=N+1;
         Nodes(:,:,N) = NewNode;
         ctc = D_Cost+ NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc + norm(Goal_Point - NewNode);
         site = Position(NewNode);
         NodesInfo(:,:,N) = [N,ParentNodeCount,ctc,F_Cost,site];
     else
         ctc = D_Cost + NodesInfo(1,3,ParentNodeCount);
         F_Cost = ctc+ norm(Goal_Point - NewNode);
         K = find(site_new == NodesInfo(1,5,:));
         Pnew = NodesInfo(1,2,K);
         Cnew = NodesInfo(1,3,K);
         Fnew = NodesInfo(1,4,K);
         if (Fnew<F_Cost)
             tmp = Pnew;
             min_F = Fnew;
             min_C = Cnew;
         else
             tmp = ParentNodeCount;
             min_F = F_Cost;
             min_C = ctc;
         end
         NodesInfo(:,:,K) = [K, tmp,min_C,min_F,site_new];
     end
     end
     end 
     
     ParentNodeCount = ParentNodeCount+1;      
end

ParentNodeCount = ParentNodeCount - 1;
i=2;
path(:,:,1) = CurrentNode;
Number = ParentNodeCount;
Trail =[];
Trail(:,:,1)= [1,Number];

while  ~isequal(CurrentNode,Start_Point)
    Q = NodesInfo(1,2,Number);
    CurrentNode = Nodes(:,:,Q);
    path(:,:,i) = Nodes(:,:,Q);
    Trail(:,:,i)= [Number,Q];
    Number = NodesInfo(1,2,Q);
    i = i+1; 
end

for k = 1:i-1
    TrailX(k) = path(1,1,k);
    TrailY(k) = path(1,2,k);
end

drawnow
plot(TrailX,TrailY,'r','linewidth',2);
hold off
toc

%% Local Planner using Quintic Fifth order Polynomial method
hold on
global x y x0 y0  m a b c n s dist L H 

x0 = 14; 
y0 = 20;
m = 0;
n = 0;
dist = 0.05; 
s = 0.005; 
L=0.8;
H=0.5;

Nodes =[];
Node_New =[];

Nodes(:,:,1)= [x0,y0];


%Main


plot(x0, y0,'s','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r')

XCar = [x0-L/2,x0-L/2,x0+L/2,x0+L/2,x0-L/2];
YCar = [y0-H/2,y0+H/2,y0+H/2,y0-H/2,y0-H/2];
% plot(XCar,YCar,'k');
title('Path Generation')
hold on

x = x0; 
y = y0; 

while (x0 > 0.00)   % Initial Sanity check

 x0 = normrnd(x0,s);
 y0 = normrnd(y0,s);
 m = normrnd(m,s); 
 
 a = (n/(2*x0^3)) - 3*m/(x0^4)+ 6*y0/(x0^5);
 b = m/(x0^3) - 3*y0/(x0^4);
 c = y0/(x0^3);  
 
 
 x = x0 - dist/(sqrt(1+m^2));
 y = y0 - (m*dist)/(sqrt(1+m^2)); 
 
 Node_New = [x,y];
 Nodes = cat(3,Nodes,Node_New);
 
 plot(x,y,'o','MarkerEdgeColor','g','MarkerSize',3,'MarkerFaceColor','b')
 drawnow
  
%  XCar_moving = [x-L/2,x-L/2,x+L/2,x+L/2,x-L/2];
%  YCar_moving = [y-H/2,y+H/2,y+H/2,y-H/2,y-H/2];
%  plot(XCar_moving,YCar_moving);
 
 x = normrnd(x,s);
 y = normrnd(y,s);
 m = normrnd(m,s); 
 
 n = 6*a*x*(x-x0)^2 + 12*a*(x^2)*(x-x0)+6*b*x*(x- x0) + 2*a*(x^3) + 6*b*(x^2) + 6*(c*x);
 
 m = m + n*(x - x0); 
 
 x0=x;
 y0=y;
end

if x0<0
plot(x, y,'s','MarkerSize',15,'MarkerFaceColor','g','MarkerEdgeColor','g')

XCar = [x0-L/2,x0-L/2,x0+L/2,x0+L/2,x0-L/2];
YCar = [y0-H/2,y0+H/2,y0+H/2,y0-H/2,y0-H/2];
% plot(XCar,YCar,'k');

end

%% Functions used in Global Planner

function [site]= Position(NewNode)
a=floor(NewNode(1));
b=floor(NewNode(2));
site = strcat(num2str(a),num2str(b));
site = str2num(site);
end

function [In_Obs, On_Border,On_Obs] = Check_Obstacle(Node)
Res=0.1;
X = Node(1);
Y = Node(2);

%Table1
XTable1 = [[-600,-600,-650,-650,-600]+750]*Res;
YTable1 = [[-500,-350,-350,-500,-500]+500]*Res;
[InTable1,OnTable1] = inpolygon(X,Y,XTable1,YTable1);

%Table2
XTable2 = [[-400,-400,-450,-450,-400]+750]*Res;
YTable2 = [[-500,-350,-350,-500,-500]+500]*Res ;
[InTable2,OnTable2] = inpolygon(X,Y,XTable2,YTable2);

%Table3
XTable3 = [[-200,-200,-250,-250,-200]+750]*Res;
YTable3 = [[-500,-350,-350,-500,-500]+500]*Res ;
[InTable3,OnTable3] = inpolygon(X,Y,XTable3,YTable3);

%Table4
XTable4 = [[0 0 50 50 0]+750]*Res;
YTable4 = [[-500,-350,-350,-500,-500]+500]*Res ;
[InTable4,OnTable4] = inpolygon(X,Y,XTable4,YTable4);

%Conference Table
XCTable = [[-600,-600,-650,-650,-600]+750]*Res;
YCTable = [[-500,-350,-350,-500,-500]+500+500]*Res;
[InCTable,OnCTable] = inpolygon(X,Y,XCTable,YCTable);

%Conference Table 1
%Conference Table 1
XCTable1 = [[-400,-400,-450,-450,-400]+750]*Res;
YCTable1=  [[-500,-350,-350,-500,-500]+500+500]*Res;
[InCTable1,OnCTable1] = inpolygon(X,Y,XCTable1,YCTable1);

%Dining Table
XDTable = [[-200,-200,-250,-250,-200]+750]*Res;
YDTable = [[-500,-350,-350,-500,-500]+500+500]*Res ;
[InDTable,OnDTable] = inpolygon(X,Y,XDTable,YDTable);

XDTable1 = [[0 0 50 50 0]+750]*Res;
YDTable1= [[-500,-350,-350,-500,-500]+500+500]*Res ;
[InDTable1,OnDTable1] = inpolygon(X,Y,XDTable1,YDTable1);


%Scene Boundaries
XWkspace = [[-750,-750 750,750,-750]+750]*Res;
YWkspace = [[-500,500,500,-500,-500]+500]*Res;
[InWk,OnWk] = inpolygon(X,Y,XWkspace,YWkspace);

in =   InTable1 | InTable2 | InTable3 | InTable4 |InCTable | InCTable1 | InDTable | InDTable1|(~InWk);
on =   OnTable1 | OnTable2 | OnTable3 | OnTable4| OnCTable | OnCTable1 | OnDTable | OnDTable1 ;
On_Obs = on;

In_Obs = in;
On_Border = OnWk;
end

function[StatusRight,NewNode] = ActionRight(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs || (x == 250)
    NewNode = CurrentNode ;
    StatusRight= 0;
else
    x =x +1;
    NewNode = [x,y];
    StatusRight = 1;
end
end

function[StatusTopRight,NewNode] = ActionTopRight(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs || (x == 250)||(y==150)
    NewNode = CurrentNode ;
    StatusTopRight= 0;
else
    x=x +1;
    y=y+1;
    NewNode = [x,y];
    StatusTopRight = 1;
end
end

function[StatusTop,NewNode] = ActionTop(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs ||(y==150)
    NewNode = CurrentNode ;
    StatusTop= 0;
else
    y=y+1;
    NewNode = [x,y];
    StatusTop = 1;
end
end

function[StatusTopLeft,NewNode] = ActionTopLeft(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs ||(x==0)||(y==150)
    NewNode = CurrentNode ;
    StatusTopLeft= 0;
else
    x=x-1;
    y=y+1;
    NewNode = [x,y];
    StatusTopLeft = 1;
end
end

function[StatusLeft,NewNode] = ActionLeft(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs ||(x==0)
    NewNode = CurrentNode ;
    StatusLeft= 0;
else
    x=x-1;
    NewNode = [x,y];
    StatusLeft = 1;
end
end

function[StatusDownLeft,NewNode] = ActionDownLeft(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs ||(x==0)||(y==0)
    NewNode = CurrentNode ;
    StatusDownLeft= 0;
else
    x=x-1;
    y=y-1;
    NewNode = [x,y];
    StatusDownLeft = 1;
end
end

function[StatusDown,NewNode] = ActionDown(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs||(y==0)
    NewNode = CurrentNode ;
    StatusDown= 0;
else
    y=y-1;
    NewNode = [x,y];
    StatusDown = 1;
end
end

function[StatusDownRight,NewNode] = ActionDownRight(CurrentNode)
x = CurrentNode(1,1);
y = CurrentNode(1,2);
Node = [x,y];
[In_Obs,On_Obs] = Check_Obstacle(Node);
if In_Obs || On_Obs||(x==250)||(y==0)
    NewNode = CurrentNode ;
    StatusDownRight= 0;
else
    x=x+1;
    y=y-1;
    NewNode = [x,y];
    StatusDownRight = 1;
end
end

