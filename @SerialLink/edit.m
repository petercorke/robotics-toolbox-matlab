        %SerialLink.edit Edit kinematic and dynamic parameters of a seriallink manipulator
        %
        % R.edit displays the kinematic parameters of the robot as an editable
        % table in a new figure.
        %
        % R.edit('dyn') as above but also displays the dynamic parameters.
        %
        % Notes::
        % - The 'Save' button copies the values from the table to the SerialLink
        %   manipulator object.  
        % - To exit the editor without updating the object just
        %   kill the figure window.
        function edit(r, dyn)
        
            isdyn = nargin > 1 && strcmp(dyn, 'dyn') == 1;

            f = figure('Position',[100 100 600 220], ...
                'Menubar', 'none', ...
                'Name', r.name);
            dh = zeros(r.n,0);
            for j=1:r.n
                L = r.links(j);
               
                %dh(j,1) = L.theta;
                dh(j,2) = L.d;
                dh(j,3) = L.a;
                dh(j,4) = L.alpha;
                dh(j,5) = L.sigma;
                dh(j,6) = L.offset;
                if ~isempty(L.qlim)
                    dh(j,7) = L.qlim(1);
                    dh(j,8) = L.qlim(2);
                else
                    dh(j,7) = -Inf;
                    dh(j,8) = Inf;
                end
                if isdyn
                    dh(j,9) = L.m;
                    dh(j,10) = L.Jm;
                    dh(j,11) = L.B;
                    dh(j,12) = L.G;
                    dh(j,13) = L.Tc(1);
                    dh(j,14) = L.Tc(2);
                    dh(j,15) = L.r(1);
                    dh(j,16) = L.r(2);
                    dh(j,17) = L.r(3);
                    dh(j,18) = L.I(1,1);
                    dh(j,19) = L.I(2,2);
                    dh(j,20) = L.I(3,3);
                    dh(j,21) = L.I(1,2);
                    dh(j,22) = L.I(2,3);
                    dh(j,23) = L.I(3,1);
                end
            end
            headings = {'theta', 'd', 'a', 'alpha', 'sigma', 'offset', 'q_min', 'q_max'};
            if isdyn
                headings = [headings 'mass', 'Jm', 'B', 'G', 'Tc+', 'Tc-', 'rx', 'ry', 'rz', 'Ixx', 'Iyy', 'Izz', 'Ixy', 'Iyz', 'Ixz'];
            end
            t = uitable(f, ...
                'ColumnEditable', true, ...
                'ColumnName', headings,...
                'Rowstriping', 'on', ...
                'Data', dh, ...
                'UserData', r);
            % Set width and height
            f.Position(3) = t.Extent(3)+50;
            f.Position(4) = t.Extent(4)+50;
            t.Position(3) = t.Extent(3)+50;
            t.Position(4) = t.Extent(4)+50;
            t.Position(1) = 0;
            t.Position(2) = 0;
            b = uicontrol('Style', 'pushbutton', ...
                'Position', [10 10 50 20], ...
                'String', 'Save', ...
                'Callback', {@edit_save, t});
        end
        
        function edit_save(button, event, table)
            
            dh = get(table, 'Data');
            r = get(table, 'UserData');
            for j=1:r.n
                L = r.links(j);
               
                %dh(j,1) = L.theta;
                L.d = dh(j,2);
                L.a = dh(j,3); 
                L.alpha = dh(j,4);
                L.sigma = dh(j,5);
                L.offset = dh(j,6);
                L.qlim(1) = dh(j,7);
                L.qlim(2) = dh(j,8);
                if numcols(dh) > 6
                    L.m = dh(j,9);
                    L.Jm = dh(j,10);
                    L.B = dh(j,11);
                    L.G = dh(j,12);
                    L.Tc(1) = dh(j,13);
                    L.Tc(2) = dh(j,14);
                    L.r(1) = dh(j,15);
                    L.r(2) = dh(j,16);
                    L.r(3) = dh(j,17);
                    L.I(1,1) = dh(j,18);
                    L.I(2,2) = dh(j,19);
                    L.I(3,3) = dh(j,20);
                    L.I(1,2) = dh(j,21);
                    L.I(2,3) = dh(j,22);
                    L.I(3,1) = dh(j,23);
                    L.I(2,1) = dh(j,21);
                    L.I(3,2) = dh(j,22);
                    L.I(1,3) = dh(j,23);
                end
            end
        end
        
