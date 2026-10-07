% similar to robotA
% with option to not compose
function T = structA(robotstruct,llist,allq, notcompose)
    if nargin < 4
        notcompose = 0;
    end
    if notcompose == 1
        To = cell(length(llist),1);
        for I=1:length(llist)
            li = robotstruct.links{I};
            if isfield(li,'rel')
                T = li.rel;
            else
                T = eye(4);
            end            
            switch(li.type)
                case 'fixed'
                case 'revolute'
                    T = T*angvec2tr(allq(li.joint),li.axis);
                otherwise
                    error('Not Implemented');
            end
            To{I} = T;
        end
        T = To;
    else
        if length(llist) > 1
            T = eye(4);
            for I=1:length(llist)
                % check if has real joint and solve it
                li = robotstruct.links{llist(I)};
                if li.joint < 1
                    q = 0;
                else
                    q = allq(li.joint);
                end
                T = T*structA(robotstruct,llist(I),q);
            end
        else
            j = llist;
            q = allq;
            
            li = robotstruct.links{j};
            if isfield(li,'rel')
                T = li.rel;
            else
                T = eye(4);
            end
            switch(li.type)
                case 'fixed'
                case 'revolute'
                    T = T*angvec2tr(q,li.axis);
                otherwise
                    error('Not Implemented');
            end        
        end
    end
    