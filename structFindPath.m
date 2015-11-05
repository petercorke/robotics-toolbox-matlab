function p = structFindPath(robotstruct,start,target)

% finds the path of link indices to go from start to end
% the path can be used with structA(robotstruct,p,allq)
A = robotstruct.linkparent;
A = (A' + A)*0.5; % symmetric

unusedNodes=ones(1,size(A,1));
emptyPath=[];

p = findpaths(A,unusedNodes,emptyPath,start,target);

end

% http://stackoverflow.com/questions/28824640/find-all-possible-paths-in-a-graph-using-matlab-brute-force-search
function paths = findpaths(Adj, nodes, currentPath, start, target)
   paths = {};
   nodes(start) = 0;
   currentPath = [currentPath start];
   childAdj = Adj(start,:) & nodes;
   childList = find(childAdj);
   childCount = numel(childList);
   if childCount == 0 || start == target
      if start == target
         paths = [paths; currentPath];
      end
      return;
   end
   for idx = 1:childCount
      currentNode = childList(idx);
      newNodes = nodes;
      newNodes(currentNode) = 0;
      newPaths = findpaths(Adj, newNodes, currentPath, currentNode, target);
      paths = [paths; newPaths];
   end
end