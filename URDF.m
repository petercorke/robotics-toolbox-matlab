
% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
classdef URDF
    
    properties
        joints
        links
        transmissions
        props
        attr
        jseq
    end
    
    methods
        function urdf = URDF(urdffile)
            if nargin == 0
                urdffile = 'katana_400_6m180.urdf.xacro';
            end
            
            dom = xmlread(urdffile);
            
            robot = dom.getElementsByTagName('robot');
            ro = robot.item(0);
            links = URDF.getChildElementsByTagName(ro,'link');
            joints = URDF.getChildElementsByTagName(ro,'joint');
            properties = URDF.getChildElementsByTagName(ro,'property');
            urdf.attr = urdf.getattr(robot.item(0));
            
            % get the properties
            fprintf('%d properties\n', length(properties));
            
            Props = [];
            for k = 1:length(properties)
                property = properties(k);
                if property.hasAttributes
                    attributes = property.getAttributes;
                    
                    att = [];
                    for i=1:attributes.getLength
                        
                        attribute = attributes.item(i-1);
                        n = char(attribute.getName);
                        v = char(attribute.getValue);
                        att.(n) = v
                    end
                    Props = [ Props; att];
                end
            end
            urdf.props = Props;
            
            % get the joints
            urdf.joints = URDF.get_elements(dom, 'joint', Props);
            
            % get the links
            urdf.links = URDF.get_elements(dom, 'link', Props);
            
            % get the transmissions
            urdf.transmissions = URDF.get_elements(dom, 'transmission', Props);
            

            p = zeros(1, length(urdf.links));
            
            for j=1:length(urdf.joints)
                    i = urdf.ln2i( urdf.joints{j}.parent.link);
                    p(i) = p(i) + 1;
                    i = urdf.ln2i( urdf.joints{j}.child.link);
                    p(i) = p(i) - 1;
            end

            
            [~,base_link] = max(p);
            base_link_name = urdf.links{base_link}.name; 
            link = base_link_name;
            for j=1:length(urdf.joints)
                jj = urdf.findnextjoint(link);
                if isempty(jj)
                    break;
                end
                urdf.jseq(j) = jj;
                link = urdf.joints{jj}.child.link;
            end
        end

        % So far Link/SerialLink is not enough expressive (really?) for supporting URDF
        % the following code exports a links and joints structure that can
        % be easily solved. This structure separates links from joints
        % allowing to hide fixed joints
        function r = asstructure(urdf)
            links = {};
            njoints = 0;
            linksbyname = [];    
            linkparent = zeros(urdf.nlinks);
            linkjoint = zeros(urdf.nlinks);
            joints = [];
            for i=1:urdf.nlinks
                linksbyname.(urdf.links{i}.name) = i;
            end
            for i=1:urdf.njoints
                T = rpy2tr(urdf.joints{i}.origin.rpy);
                T(1:3,4) = urdf.joints{i}.origin.xyz;
                s = [];
                s.rel = T;
                s.type = urdf.joints{i}.type;
                s.name = urdf.joints{i}.child.link;
                s.parent = urdf.joints{i}.parent.link;
                s.id = length(links)+1;
                s.parentid = linksbyname.(s.parent);
                
                if isfield(urdf.joints{i},'axis')
                    s.axis = urdf.joints{i}.axis.xyz;
                end
                
                switch(urdf.joints{i}.type)
                    case 'revolute'
                        njoints = njoints+1;
                        s.joint = njoints;
                        joints.(s.name) = njoints;
                        linkjoint(s.id,njoints) = 1;
                    case 'fixed'
                        s.joint = 0;
                    otherwise
                        error(['Joint type unsupported ' urdf.joints{i}.type]);
                end                
                linkparent(s.id,s.parentid) = 1;
                links{end+1} = s;
                linksbyname.(s.name) = s.id;
            end
            linkjoint = linkjoint(:,1:njoints);
            r = [];
            r.name = urdf.attr.name;
            r.links = links; % struct of links
            r.jointsbyname = joints; % struct child link name => joint identifier
            r.njoints = njoints;
            r.linksbyname = linksbyname; % links byname
            r.linkjoint = linkjoint;
            r.linkparent = linkparent; % parent relationship
        end

        function r = robot(urdf)
            links = [];
            for i=1:urdf.njoints
                switch(urdf.joints{i}.type)
                    otherwise
                        error(['Joint type unsupported ' urdf.joints{i}.type]);
                end
            end
            r = SerialLink(links, 'name', urdf.attr.name);
        end
        
        function n = nlinks(urdf)
            n = length(urdf.links);
        end
        
        function n = njoints(urdf)
            n = length(urdf.joints);
        end
        
        % map joint/link name to an index
        
        function joint = findnextjoint(urdf, link)
            joint = [];
            for j=1:length(urdf.joints)
                if strcmp(link, urdf.joints{j}.parent.link)
                    joint = j;
                    return
                end
            end
        end
        
        function idx = ln2i(urdf, name)
            idx = [];
            for i=1:length(urdf.links)
                if strcmp(urdf.links{i}.name, name)
                    idx = i;
                    return
                end
            end
        end
        
        function display(urdf)
            for j=1:length(urdf.joints)
                joint = urdf.joints{j};
                fprintf('j%d: %s -> %s (%s)\n', j, joint.parent.link, joint.child.link, joint.type);
            end
        end
    end
    
    methods (Static)
       function r = getChildElementsByTagName(node,name)
            r = [];
            nodeChildren = node.getChildNodes;
            for i=1:nodeChildren.getLength
                child = nodeChildren.item(i-1);
                if child.getNodeType == 3
                    continue;
                end
                if strcmp(char(child.getNodeName),name)
                    r = [r; child];
                end
            end
        end
        
        function List = get_elements(doc, elname, props)
            elements  = URDF.getChildElementsByTagName(doc.getDocumentElement(),elname);
            % get the links
            List = {};
            fprintf('%d %s\n', length(elements), elname);
            % step through the list of  elements found
            for k = 1: length(elements)
                element = elements(k);
                
                info = URDF.descend(element, props);
                info = URDF.getattr(element, info, props);
                List{k} = info;
            end
        end
        
        function t = descend(node, props)
            if node.hasChildNodes
                t = [];
                
                nodeChildren = node.getChildNodes;
                for i=1:nodeChildren.getLength
                    child = nodeChildren.item(i-1);
                    if child.getNodeType == 3
                        continue;
                    end
                    result = URDF.descend(child, props);
                    if ~isempty(result)
                        n = char(child.getNodeName);
                        t.(n) = result;
                    end
                end
            elseif node.hasAttributes
                t = URDF.getattr(node, [], props);
            else
                t = [];
            end
        end
        




        function att = getattr(node, att, props)
            if nargin < 2
                att = [];
            end
            if nargin < 3
                props = [];
            end
            
            attributes = node.getAttributes;
            for i=1:attributes.getLength
                attribute = attributes.item(i-1);
                n = char(attribute.getName);
                v = char(attribute.getValue);
                
                % skip properties with colon in them
                n = strrep(n, ':', '_');
                
                %do simple xacro type substitution
                %xyz="0 0 ${-base_height}"
                if ~isempty(props)
                    
                    % is there a ${...} substring
                    [s,e] = regexp(v, '\${[^}]*');
                    if ~isempty(s)
                        macro = v(s+2:e);
                        %fprintf('XACRO: %s --> ', v);
                        % if yes, then regexprep replace within it for every property in the list
                        for k=1:length(props)
                            macro = strrep(macro, props(k).name, props(k).value);
                        end
                        % stick the macro back into the original string
                        macro = strrep(macro, ' ', '');  % remove all blanks
                        v = [v(1:s-1) macro v(e+2:end)];
                        %fprintf('%s\n', v);
                    end
                end
                
                % attempt to convert it to a scalar or vector, fallback is a string
                att.(n) = str2num(v);
                if isempty( att.(n) )
                    att.(n) = v;
                end
            end
        end
    end
end

