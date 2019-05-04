% Dimensions for workspace - 250 x 150
% 8Connected Workspace
Nodes = [];
NodesInfo = [];

CurrentNode= [1,1];
%Final_Point = 


Nodes(:,:,1) = CurrentNode

%NodesInfo = Node#, Parent Node, Cost 
NodesInfo(:,:,1) = [1,0,0];
nodeno =2;
%% Main Loop
for  i = 1:50000
     disp(i)
     
     CurrentNode = Nodes(:,:,i);
     
     [s9,NewNode] = CheckObstacle(CurrentNode); % Checking if Current Node lies in Obstacle Space
     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [s1,NewNode] = Left(CurrentNode); %Output from Action "Left"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s1=0;
        end
    end
    if s1 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1];
        nodeno=nodeno+1;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [s5,NewNode] = TopLeft(CurrentNode); %Output from Action "TopLeft"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s5=0;
        end
    end
    if s5 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1.41];
        nodeno=nodeno+1;
    end   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    [s3,NewNode] = Up(CurrentNode); %Output from Action "Up"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s3=0;
        end
    end
    if s3 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1];
        nodeno=nodeno+1;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    [s6,NewNode] = TopRight(CurrentNode);
    %Output from Action "TopRight"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s6=0;
        end
    end
    if s6 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1.41];
        nodeno=nodeno+1;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    [s2,NewNode] = Right(CurrentNode); % Output from Action "Right"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s2=0;
        end
    end
    if s2 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1];
        nodeno=nodeno+1;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    [s8,NewNode] = DownRight(CurrentNode); % Output from Action "DownRight"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s8=0;
        end
    end
    if s8 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1.41];
        nodeno=nodeno+1;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    [s4,NewNode] = Down(CurrentNode); %Output from Action "Down"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s4=0;
        end
    end
    if s4 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1];
        nodeno=nodeno+1;
    end
       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
                
    [s7,NewNode] = Down(CurrentNode); %Output from Action "DownLeft"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s7=0;
        end
    end
    if s7 == 1 && s9 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,1.41];
        nodeno=nodeno+1;
    end   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
end


    
   


%% Functions

% Finding Location of Point

function [x,y] = Current_Location(CurrentNode)
[row, col] = CurrentNode;
x = row;
y = col;
end

% Functions for the Actions , Checking Obstacles

function [s1,NewNode] = Left(CurrentNode)
[x,y]= Current_Location(CurrentNode);

newx=x-1;
newy=y;
if x==0
    s1=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s1=1;
end

function [s2,NewNode] = Right(CurrentNode)
[x,y]= Current_Location(CurrentNode);

newx=x+1;
newy=y;
if x==250
    s2=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s2=1;
end

function [s3,NewNode] = Up(CurrentNode)
[x,y]=Current_Location(CurrentNode);

newx=x;
newy=y+1;
if y==150
    s3=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s3=1;

end

function [s4,NewNode] = Down(CurrentNode)
[x,y]=Current_Location(CurrentNode);

newx=x;
newy=y-1;
if y==0
    s3=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s4=1;

end

function [s5,NewNode] = TopLeft(CurrentNode)
[x,y]=Current_Location(CurrentNode);

newx=x-1;
newy=y+1;
if x==0||y==150
    s5=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s5=1;

end

function [s6,NewNode] = TopRight(CurrentNode)
[x,y]=Current_Location(CurrentNode);

newx=x+1;
newy=y+1;
if x==250||y==150
    s6=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s6=1;

end

function [s7,NewNode] = DownLeft(CurrentNode)
[x,y]=Current_Location(CurrentNode);

newx=x+1;
newy=y+1;
if x==0||y==0
    s7=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s7=1;

end

function [s8,NewNode] = DownRight(CurrentNode)
[x,y]=Current_Location(CurrentNode);

newx=x+1;
newy=y-1;
if x==250||y==0
    s8=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s8=1;

end

function [s9,NewNode] = CheckObstacle(CurrentNode)
[x,y] = Current_Location(CurrentNode);

if x > 55 || x <105 && y > 67.5 || y < 112.5
    s9 = 0;
    NewNode = CurrentNode;
    
elseif 41*x + 25*y -6295 > 0 && y >14 && 2*x + 19*y -1285 < 0 && 38*x -7*y -5647 > 0 && 38*x +23*y -8317 < 0 && 37*x -23*y -5936 < 0
        s9 = 0;
        NewNode = CurrentNode;
        
elseif (x-180)^2 + (y-120)^2  < 225
       s9 = 0;
       NewNode = CurrentNode;
else s9 =1;
end
end
   