%Initial Conditions //Specify START AND GOAL POINTS HERE ---->
Start_Point =[10,1];
Goal_Point =[160,100];

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

%Plotting Scene
%Circular Obstacle
Xc=180;
Yc=120;
t = 0:0.01:2*pi;
x = 15*cos(t)+ Xc;
y = 15*sin(t) +Yc;
drawnow
plot(x,y), hold on
grid on
fill(x,y,'b');

%Square Obstacle
XSq = [ 55,105,105,55,55];
YSq = [ 67.5,67.5,112.5,112.5,67.5] ;
drawnow
plot(XSq,YSq);
fill(XSq,YSq,'b');

%Polygon Obstacle
XPol= [145,168,188,165,158,120,145];
YPol = [14,14,51,89,51,55,14];
drawnow
plot(XPol,YPol);
fill(XPol,YPol,'b');

%Scene Boundary
xp3 = [0,250 250,0,0];
yp3 = [0,0,150,150,0];
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
save('Path.mat','path')
function [site]= Position(NewNode)
a=floor(NewNode(1));
b=floor(NewNode(2));
site = strcat(num2str(a),num2str(b));
site = str2num(site);
end

function [In_Obs, On_Border,On_Obs] = Check_Obstacle(Node)

X = Node(1);
Y = Node(2);

XCentre=180;
YCentre = 120;
t = 0:0.01:2*pi;
XCircle = 15*cos(t)+ XCentre;
YCircle = 15*sin(t)+ YCentre;
[InCircle,OnCircle] = inpolygon(X,Y,XCircle,YCircle);

XSquare = [ 55,105,105,55,55];
YSquare = [ 67.5,67.5,112.5,112.5,67.5] ;
[InSquare,OnSquare] = inpolygon(X,Y,XSquare,YSquare);

XPoly = [145,168,188,165,158,120,145];
YPoly = [14,14,51,89,51,55,14];
[InPoly,OnPoly] = inpolygon(X,Y,XPoly,YPoly);

XWkspace = [0,250,250,0,0];
YWkspace = [0,0,150,150,0];

[InWk,OnWk] = inpolygon(X,Y,XWkspace,YWkspace);

in = InCircle | InSquare | InPoly | (~InWk);
on = OnCircle | OnSquare | OnPoly;
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
