
Nodes = [];
NodesInfo = [];
j=1;



Nodes(:,:,1) = [1,2,3;7,8,0;4,5,6];
%NodesInfo = Node #, Parent Node #, Cost to come
NodesInfo(:,:,1) = [1,0,0];

NodeSet.Nodes = Nodes;
NodeSet.NodesInfo = NodesInfo;

nodeno=2;

%% Main Loop

% Update for Node Count and NodeInfo Data
for i=1:100
    disp(i)
    
    CurrentNode = Nodes(:,:,i);
    
    [s1,NewNode] = Left(CurrentNode); %Output from Action "Left"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s1=0;
        end
    end
    if s1 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,0];
        nodeno=nodeno+1;
    end

    [s1,NewNode] = Right(CurrentNode); % Output from Action "Right"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s1=0;
        end
    end
    if s1 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,0];
        nodeno=nodeno+1;
    end
    
    [s1,NewNode] = Up(CurrentNode); %Output from Action "Up"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s1=0;
        end
    end
    if s1 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,0];
        nodeno=nodeno+1;
    end
    
    [s1,NewNode] = Down(CurrentNode); %Output from Action "Down"
    for j=1:(nodeno-1)
        if NewNode==Nodes(:,:,j)
            s1=0;
        end
    end
    if s1 == 1
        Nodes = cat(3,Nodes,NewNode);
        NodesInfo(:,:,nodeno) = [nodeno,i,0];
        nodeno=nodeno+1;
    end
end
%% Functions

%Finding Location of Zero Tile
function [x,y] = BlankTileLocation(CurrentNode)
X = CurrentNode;
[row,col] = find(X==0);
x = row;
y = col;
end


%Functions for the Actions
function [s1,NewNode] = Left(CurrentNode)
[x,y]=BlankTileLocation(CurrentNode);

newx=x;
newy=y-1;
if y==1
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
[x,y]=BlankTileLocation(CurrentNode);

newx=x;
newy=y+1;
if y==3
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
[x,y]=BlankTileLocation(CurrentNode);

newx=x-1;
newy=y;
if x==1
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
[x,y]=BlankTileLocation(CurrentNode);

newx=x+1;
newy=y;
if x==3
    s4=0;
    NewNode = CurrentNode;
    return
end
valtoShift=CurrentNode(x,y);
CurrentNode(x,y)=CurrentNode(newx,newy);
CurrentNode(newx,newy)=valtoShift;

NewNode= CurrentNode;
s4=1;
end

