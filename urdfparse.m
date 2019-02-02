function urdf = urdfparse(filename)
    
    root = xmlread(filename);
    
    robot = root.getElementsByTagName('robot').item(0);
    
    
    %% process all the materials elements
    % - these are optional but we do them first because link elements
    %   reference them
    materialNodes = getChildrenByTagName(robot, 'material');
    materials = [];
    materialNames = [];
    if ~isempty(materialNodes)

        for i = 1:length(materialNodes)
            node = materialNodes(i);
            materials(i).name = string(node.getAttribute('name'));
            materials(i).rgba = str2num(node.getElementsByTagName('color').item(0).getAttribute('rgba'));
            materials(i).node = node;
        end
        
        urdf.materials = materials;
        materialNames = [materials.name];
    end
    

    %% process all the link elements
    
    % create a vector of joint structures, fields are:
    % name       name of the joint
    % type       type of joint (fixed, continuous, prismatic)
    % T          SE3 object for pose of joint wrt link frame
    % xyz        position of joint wrt link frame
    % rpy        RPY angles of joint frame wrt link frame
    % node       reference to XML DOM node for this <joint>
    % child      link index for downstream link
    % parent     link index for downstream link
    
    linkNodes = getChildrenByTagName(robot, 'link');
    
    for i = 1:length(linkNodes)
        node = linkNodes(i);
        links(i).name = string(node.getAttribute('name'));
        try
            links(i).geometry = node.getElementsByTagName('geometry').item(0);
            links(i).mesh = links(i).geometry.getElementsByTagName('mesh').item(0).getAttribute('filename');
        end
        try
            links(i).material = find(strcmp((node.getElementsByTagName('material').item(0).getAttribute('name')), materialNames));
        end
        try
            inertial = node.getElementsByTagName('inertial').item(0);
            try
                links(i).m = str2num(inertial.getElementsByTagName('mass').item(0).getAttribute('value'));
            end
            try
                I = inertial.getElementsByTagName('inertia').item(0);
                links(i).I.xx = str2num(I.getAttribute('ixx'));
                links(i).I.xy = str2num(I.getAttribute('ixy'));
                links(i).I.xz = str2num(I.getAttribute('ixz'));
                links(i).I.yy = str2num(I.getAttribute('iyy'));
                links(i).I.yz = str2num(I.getAttribute('iyz'));
                links(i).I.zz = str2num(I.getAttribute('izz'));
            end
            try
                t = str2num( node.getElementsByTagName('origin').item(0).getAttribute('xyz'));
                rpy = str2num( node.getElementsByTagName('origin').item(0).getAttribute('rpy'));
                links(i).com = SE3(t) * SE3.rpy(rpy);
            end
        end
        t = [0 0 0]; rpy = [0 0 0]; % in case not provided
        try
            t = str2num( node.getElementsByTagName('origin').item(0).getAttribute('xyz'));
            rpy = str2num( node.getElementsByTagName('origin').item(0).getAttribute('rpy'));
        end
        if isempty(t) 
            t = [0 0 0]; % in case the tag has no attribute
        end
        if isempty(rpy) 
            rpy = [0 0 0];  % in case the tag has no attribute
        end
        links(i).T = SE3(t) * SE3.rpy(rpy);
        links(i).node = node;
        links(i).children = [];
        links(i).parent = [];
    end
    
    linkNames = [links.name];  % a list of link names
    
    
    %% process all the joint elements

    % create a vector of link structures, fields are are:
    % name       name of the link
    % geometry   reference to XML DOM node
    % T          SE3 object for origin of visual wrt link frame
    % node       reference to XML DOM node for this <link>
    % children   list of joint indexes attaching this link to its children
    % parent     index of joint attaching this link to its parent
    
    jointNodes = getChildrenByTagName(robot, 'joint');
    joints = [];
    for i = 1:length(jointNodes)
        node = jointNodes(i);
        joints(i).name = string(node.getAttribute('name'));
        joints(i).type = string(node.getAttribute('type'));
        try
            joints(i).axis = str2num(node.getElementsByTagName('axis').item(0).getAttribute('xyz'));
        end
        p = find(strcmp(node.getElementsByTagName('parent').item(0).getAttribute('link'), linkNames));
        c = find(strcmp(node.getElementsByTagName('child').item(0).getAttribute('link'), linkNames));
        joints(i).parent = p;
        joints(i).child = c;
        links(p).children = [links(p).children i];
        links(c).parent = i;
        
        t = [0 0 0]; rpy = [0 0 0];  % in case not provided
        try
            t = str2num( node.getElementsByTagName('origin').item(0).getAttribute('xyz'));
            rpy = str2num( node.getElementsByTagName('origin').item(0).getAttribute('rpy'));
        end
        if isempty(t)
            t = [0 0 0]; % in case the tag has no attribute
        end
        if isempty(rpy)
            rpy = [0 0 0]; % in case the tag has no attribute
        end
        joints(i).T = SE3(t) * SE3.rpy(rpy);
        joints(i).xyz = t;
        joints(i).rpy = rpy;
        joints(i).node = node;
    end
    urdf.joints = joints;
    urdf.links = links;
    
    urdf.endlinks = find( cellfun('isempty', {links.children}) );
    urdf.ets = [];
    
    %% create a transform list
    
    % the mechanism might have multiple endpoints (if branched), for each
    % endpoint compute an ETS chain and return a string array
    
    for endlink = urdf.endlinks        
        transforms = "";
        joint = joints(links(endlink).parent);
        while true
            transforms = xform2s(joint) + axis2s(joint) + transforms;
            joint = joints(links(joint.parent).parent);
            if isempty(joint)
                break;
            end
        end
        transforms = strtrim(transforms);
        urdf.ets = [urdf.ets transforms];
    end
    
    %% display a summary
    fprintf('\nLinks::\n');    
    for i=1:length(links)
        name = links(i).name;
        if ismember(i, urdf.endlinks)
            name = "*" + name;
        end
        geomlist = "";
        if ~isempty(links(i).geometry)
            g = links(i).geometry.getChildNodes;
            for k=0:g.getLength-1
                c = g.item(k);
                if c.getNodeType == 1  % skip text nodes
                    if geomlist == ""
                        geomlist = "shape=";
                    else
                        geomlist = geomlist + ", ";
                    end
                    geomlist = geomlist + string(c.getNodeName);
                end
            end
        end
        fprintf('%2d: %10s; parentjoint=%s, childjoints=%s; %s\n', i, name, ...
            formatj(links(i).parent), formatj(links(i).children), geomlist);
    end
    if ~isempty(joints)
        fprintf('\nJoints::\n');
        for i=1:length(joints)
            fprintf('%2d: %10s; type=%s, parentlink=%d, childlink=%d\n', i, joints(i).name, joints(i).type, joints(i).parent, joints(i).child);
        end
    end
    if ~isempty(materials)
        fprintf('\nMaterials::\n');
        for i=1:length(materials)
            fprintf('%10s: %f %f %f %f\n', materials(i).name, materials(i).rgba);
        end
    end
    if strlength(transforms) > 0
        fprintf('\nForward kinematics::\n  %s\n', transforms);
    end

end

function s = formatj(j)
    if isempty(j)
        s = '-';
    else
        s = strip(num2str(j, '%d,'), 'right', ',');
    end
end

function t = xform2s(joint)
    
    t = "";
    if joint.xyz(1) ~= 0
        t = t + sprintf('Tx(%.12g) ', joint.xyz(1));
    end
    if joint.xyz(2) ~= 0
        t = t + sprintf('Ty(%.12g) ', joint.xyz(2));
    end
    if joint.xyz(3) ~= 0
        t = t + sprintf('Tz(%.12g) ', joint.xyz(3));
    end
    if joint.rpy(3) ~= 0
        t = t + sprintf('Rz(%.12g) ', round(joint.rpy(3)*180/pi, 3));
    end
    if joint.rpy(2) ~= 0
        t = t + sprintf('Ry(%.12g) ', round(joint.rpy(2)*180/pi,3));
    end
    if joint.rpy(1) ~= 0
        t = t + sprintf('Rx(%.12g) ', round(joint.rpy(1)*180/pi, 3));
    end
end

function t = axis2s(joint)
    
    t = "";
    switch joint.type
        case {'revolute', 'continuous'}
            assert(length(find(joint.axis)) == 1, 'Joint axis must be about x, y or z only');
            if joint.axis(1) ~= 0
                t = t + sprintf('Rx(%s) ', joint.name);
            end
            if joint.axis(2) ~= 0
                t = t + sprintf('Ry(%s) ', joint.name);
            end
            if joint.axis(3) ~= 0
                t = t + sprintf('Rz(%s) ', joint.name);
            end
        case 'prismatic'
            assert(length(find(joint.axis)) == 1, 'Joint axis must be about x, y or z only');
            if joint.axis(1) ~= 0
                t = t + sprintf('Tx(%s) ', joint.name);
            end
            if joint.axis(2) ~= 0
                t = t + sprintf('Ty(%s) ', joint.name);
            end
            if joint.axis(3) ~= 0
                t = t + sprintf('Tz(%s) ', joint.name);
            end
        case 'fixed'
    end
   
end

function children = getChildrenByTagName(parent, name)
    % return array of immediate child nodes with node name equal to name
    % REF https://stackoverflow.com/questions/18776408/get-all-the-children-for-a-given-xml-in-java
    children = [];
    childnodes = parent.getChildNodes;
    for i=1:childnodes.getLength
        node = childnodes.item(i-1);
        if node.getNodeType == node.ELEMENT_NODE && strcmp(node.getNodeName, name)
            children = [children node];
        end
    end
end
    