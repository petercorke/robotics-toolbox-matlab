function urdf = urdfparse(filename)
    
    root = xmlread(filename);
    
    robot = root.getChildNodes.item(0);
    
    
    %% process all the materials elements
    % - these are optional but we do them first because link elements
    %   reference them
    materialNodes = robot.getElementsByTagName('material');
    materials = [];
    i = 1;
    for j=1:materialNodes.getLength
        node = materialNodes.item(j-1);
        if isEqualNode(node.getParentNode, robot)
            % only process material nodes at the top level
            materials(i).name = string(node.getAttribute('name'));
            materials(i).rgba = str2num(node.getElementsByTagName('color').item(0).getAttribute('rgba'));
            materials(i).node = node;
            i = i + 1;
        end
    end
    if ~isempty(materials)
        urdf.materials = materials;
        materialNames = [materials.name];
    end
    

    %% process all the link elements
    linkNodes = robot.getElementsByTagName('link');
    for i=1:linkNodes.getLength
        node = linkNodes.item(i-1);
        links(i).name = string(node.getAttribute('name'));
        links(i).geometry = node.getElementsByTagName('geometry').item(0);
        try
            links(i).material = find(strcmp((node.getElementsByTagName('material').item(0).getAttribute('name')), materialNames));
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
    end
    
    linkNames = [links.name];  % a list of link names
    urdf.links = links;
    
    %% process all the joint elements
    jointNodes = robot.getElementsByTagName('joint');
    joints = [];
    for i=1:jointNodes.getLength
        node = jointNodes.item(i-1);
        joints(i).name = string(node.getAttribute('name'));
        joints(i).type = string(node.getAttribute('type'));
        try
            joints(i).axis = str2num(node.getElementsByTagName('axis').item(0).getAttribute('xyz'));
        end
        joints(i).parent = find(strcmp(node.getElementsByTagName('parent').item(0).getAttribute('link'), linkNames));
        joints(i).child = find(strcmp(node.getElementsByTagName('child').item(0).getAttribute('link'), linkNames));
        
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
    
    %% create a transform list
    transforms = "";

    for joint = joints
        transforms = transforms + xform2s(joint) + axis2s(joint);
    end
    
    transforms = strtrim(transforms);
    urdf.ets = char(transforms);
    
    %% display a summary
    fprintf('\nLinks::\n');    
    for i=1:length(links)
        fprintf('%2d: %10s; %s\n', i, links(i).name, links(i).geometry);
    end
    if ~isempty(joints)
        fprintf('\nJoints::\n');
        for i=1:length(joints)
            fprintf('%2d: %10s; type=%s, parent=%d, child=%d\n', i, joints(i).name, joints(i).type, joints(i).parent, joints(i).child);
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

function t = xform2s(joint)
    
    t = "";
    if joint.xyz(1) ~= 0
        t = t + sprintf('Tx(%g) ', joint.xyz(1));
    end
    if joint.xyz(2) ~= 0
        t = t + sprintf('Ty(%g) ', joint.xyz(2));
    end
    if joint.xyz(3) ~= 0
        t = t + sprintf('Tz(%g) ', joint.xyz(3));
    end
    if joint.rpy(1) ~= 0
        t = t + sprintf('Rz(%g) ', joint.rpy(1));
    end
    if joint.rpy(2) ~= 0
        t = t + sprintf('Ry(%g) ', joint.rpy(2));
    end
    if joint.rpy(3) ~= 0
        t = t + sprintf('Rx(%g) ', joint.rpy(3));
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