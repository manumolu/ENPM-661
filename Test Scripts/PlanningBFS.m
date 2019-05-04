% Dimensions of Workspace - 250X150
% 8 Connected Workspace

%Setting the First Node to start Search
CurrentNode = Start_Point;

%Initializing Nodes,NodesInfo
Nodes = [];
Nodes(:,:,1) = Start_Point;
NodesInfo = [];

%NodesInfo = Node#, Parent Node, Cost 
NodesInfo(:,:,1) = [1,0,0];

%Initializing Counter
i=0;

while ~isequal(CurrentNode,Goal_Point)
    i=i+1;
    disp(i)
    CurrentNode = Nodes(:,:,i);
    [PossibleNodes] = ChildNodes(CurrentNode);
    Status1 =  CheckDuplicateNodes(PossibleNodes);
    StatusAll = CheckObstacles(PossibleNodes);
    for m = 1:8
        if Status1(m) && StatusAll(m) == 1
            Nodes = cat(3,Nodes,PossibleNodes(:,:,m));
        end
        while Status1(m) && StatusAll(m) == 1
            nodenumber = nodenumber+1;
        end
    end 
end
    










    


    
    
    
    


           
    

    


