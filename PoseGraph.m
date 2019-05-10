%PoseGraph Pose graph 

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
classdef PoseGraph < handle
    
    properties
        graph
        
        ngrid
        center
        cellsize
    end
    
    methods
        
        function pg = PoseGraph(filename, varargin)
            % parse the file data
            % we assume g2o format
            %    VERTEX* vertex_id X Y THETA
            %    EDGE* startvertex_id endvertex_id X Y THETA IXX IXY IYY IXT IYT ITT
            % vertex numbers start at 0
            
            opt.laser = false;
            
            opt = tb_optparse(opt, varargin);
            
            pg.graph = PGraph(3, 'distance', 'SE2');
            
            fp = fopen(filename, 'r');
            assert(fp > 0, 'Can''t open file %s', filename);
            
            toroformat = false;
            nlaser = 0;
            
            % indices into ROBOTLASER1 record for the 3x3 info matrix in column major
            % order
            g2o =  [6  7  8  7  9  10   8  10  11];
            toro = [6  7 10  7  8  11  10  11   9];
            
            % we keep an array pgi = vindex(gi) to map g2o vertex index to PGraph vertex index
            
            tic
            while ~feof(fp)
                
                line = fgets(fp);
                
                % is it a comment?
                if line(1) == '#'
                    continue;
                end
                
                % get keyword
                k = strfind(line, ' ');
                
                % and deal with it
                switch line(1:k-1)
                    case 'VERTEX_SE2'
                        % g2o format vertex
                        vertex = sscanf(line(k+1:end), '%d %f %f %f')';
                        v = pg.graph.add_node(vertex(2:4));
                        vindex(vertex(1)+1) = v;
                        vd.type = 'vertex';
                        pg.graph.setvdata(v, vd);
                        
                    case 'VERTEX_XY'
                        vertex = sscanf(line(k+1:end), '%d %f %f')';
                        v = pg.graph.add_node(vertex(2:4));
                        vindex(vertex(1)+1) = v;
                        vd.type = 'landmark';
                        pg.graph.setvdata(v, vd);
                        
                        
                    case 'EDGE_SE2'
                        % g2o format edge
                        edge = sscanf(line(k+1:end), '%f')';
                        v1 = vindex(edge(1)+1);
                        v2 = vindex(edge(2)+1);
                        
                        % create the edge
                        e = pg.graph.add_edge(v1, v2);
                        
                        % create the edge data as a structure
                        %  X  Y  T
                        %  3  4  5
                        ed.mean = edge(3:5);
                        
                        % IXX IXY IXT IYY IYT ITT
                        %   6   7   8   9  10  11
                        ed.info = reshape(edge(g2o), [3 3]);
                        
                        % and attach it
                        pg.graph.setedata(e, ed);
                        
                    case 'VERTEX2'
                        toroformat = true;
                        vertex = sscanf(line(k+1:end), '%d %f %f %f')';
                        v = pg.graph.add_node(vertex(2:4));
                        vindex(vertex(1)+1) = v;
                        vd.type = 'vertex';
                        pg.graph.setvdata(v, vd);
                        
                    case 'EDGE2'
                        toroformat = true;
                        edge = sscanf(line(k+1:end), '%f')';
                        v1 = vindex(edge(1)+1);
                        v2 = vindex(edge(2)+1);
                        
                        % create the edge
                        e = pg.graph.add_edge(v1, v2);
                        % create the edge data as a structure
                        %  X  Y  T
                        %  3  4  5
                        ed.mean = edge(3:5);
                        
                        % IXX IXY IYY ITT IXT IYT
                        %   6   7   8   9  10  11
                        ed.info = reshape(edge(toro), [3 3]);
                        % and attach it
                        pg.graph.setedata(e, ed);
                        
                    case 'ROBOTLASER1'
                        if ~opt.laser
                            continue;
                        end
                        
                        % laser records are associated with the immediately preceding VERTEX record
                        [laser,n] = sscanf(line(k+1:end), '%f');
                        nbeams = laser(8);
                        vd.theta = [0:nbeams-1] * laser(4) + laser(2);
                        vd.range = laser(9:8+nbeams)';
                        vd.time = laser(21+nbeams);
                        pg.graph.setvdata(v, vd);
                        nlaser = nlaser + 1;
                        
                    otherwise
                        error('RTB:posegraph:badfile', 'Unexpected line  <%s> in %s', line(1:k-1), filename);
                end
            end
            elapsed = toc;
            
            fclose(fp);
            
            if toroformat
                fprintf('loaded TORO/LAGO format file: %d nodes, %d edges in %.2f sec\n', pg.graph.n, pg.graph.ne, elapsed);
            else
                fprintf('loaded g2o format file: %d nodes, %d edges in %.2f sec\n', pg.graph.n, pg.graph.ne, elapsed);
                if nlaser > 0
                    fprintf('  %d laser scans: %d beams, fov %g to %g deg, max range %g\n', ...
                        nlaser, nbeams, [laser(2) sum(laser(2:3))]*180/pi, laser(5) );
                end
            end
        end
        
        function [r, theta] = scan(pg, n)
            vd = pg.graph.vdata(n);
            r = vd.range;
            theta = vd.theta;
        end
        
        function [X,Y] = scanxy(pg, n)
            vd = pg.graph.vdata(n);
            
            [x,y] = pol2cart(vd.theta, vd.range);
            if nargout == 1
                X = [x; y];
            elseif nargout == 2
                X = x; Y = y;
            end
        end
        
        function plot_scan(pg, n)
            for i=n(:)'
                [x,y] = pg.scanxy(i);
                plot(x, y, '.', 'MarkerSize', 10);
                pause
            end
        end
        
        function xyt = pose(pg, i)
            xyt = pg.graph.coord(i);
        end
        
        function t = time(pg, n)
            t = pg.graph.vdata(n).time;
        end
        
        function plot(pg, varargin)
            pg.graph.plot(varargin{:});
            xlabel('x')
            ylabel('y')
            grid on
        end
        
        function world = scanmap(pg, varargin)
            
            opt.center = [75 50];
            opt.ngrid = 3000;
            opt.cellsize = 0.1;
            
            pg = tb_optparse(opt, varargin, pg);
            
            h = waitbar(0, 'rendering a map');
            
            world = zeros(pg.ngrid, pg.ngrid, 'int32');
            for i=1:1:pg.graph.n
                
                if rem(i, 20) == 0
                    waitbar(i/pg.graph.n, h)
                end
                
                xy = pg.scanxy(i);
                [r,theta] = pg.scan(i);
                xy(:,r>40) = [];
                xyt = pg.graph.coord(i);
                
                xy = SE2(xyt) * xy;
                
                % start of each ray
                [x1,y1] = pg.w2g(xyt(1:2));
                
                for s=1:numcols(xy)
                    
                    % end of each ray
                    [x2,y2] = pg.w2g(xy(:,s));
                    
                    % all cells along the ray
                    p = bresenham(x1, y1, x2, y2);
                    try                    
                        k = sub2ind(size(world), p(:,1), p(:,2));
                        
                        k1 = k(1:end-1); k2 = k(end);
                        world(k1) = world(k1) - 1;
                        world(k2) = world(k2) + 1;
                    catch me
                        % come here if any point on the ray is outside the grid
                        % silently ignore it
                    end
                end
            end
            close(h)
            %idisp(world)
        end
        
        function [gx,gy] = w2g(pg, w)
            dd = 0.10;
            
            w = w(:) + pg.center(:);
            g = round(w/pg.cellsize);
            gx = g(1); gy = g(2);
        end

        function plot_occgrid(pg, w)
            
            x = [1:numcols(w)]*pg.cellsize - pg.center(1);
            y = [1:numrows(w)]*pg.cellsize - pg.center(2);
            w(w<0) = -1;
            w(w>0) = 1;
            w=-w;
            idisp(w, 'nogui', 'xydata', {x, y})
            xlabel('x'); ylabel('y');
        end
        
        
        
        %   This source code is part of the graph optimization package
        %   deveoped for the lectures of robotics2 at the University of Freiburg.
        %
        %     Copyright (c) 2007 Giorgio Grisetti, Gian Diego Tipaldi
        %
        %   It is licences under the Common Creative License,
        %   Attribution-NonCommercial-ShareAlike 3.0
        %
        %   You are free:
        %     - to Share - to copy, distribute and transmit the work
        %     - to Remix - to adapt the work
        %
        %   Under the following conditions:
        %
        %     - Attribution. You must attribute the work in the manner specified
        %       by the author or licensor (but not in any way that suggests that
        %       they endorse you or your use of the work).
        %
        %     - Noncommercial. You may not use this work for commercial purposes.
        %
        %     - Share Alike. If you alter, transform, or build upon this work,
        %       you may distribute the resulting work only under the same or
        %       similar license to this one.
        %
        %   Any of the above conditions can be waived if you get permission
        %   from the copyright holder.  Nothing in this license impairs or
        %   restricts the author's moral rights.
        %
        %   This software is distributed in the hope that it will be useful,
        %   but WITHOUT ANY WARRANTY; without even the implied
        %   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
        %   PURPOSE.
        
        
        %ls-slam.m
        %this file is released under the creative common license
        
        %solves a graph-based slam problem via least squares
        %vmeans: matrix containing the column vectors of the poses of the vertices
        %	 the vertices are odrered such that vmeans[i] corresponds to the ith id
        %eids:	 matrix containing the column vectors [idFrom, idTo]' of the ids of the vertices
        %	 eids[k] corresponds to emeans[k] and einfs[k].
        %emeans: matrix containing the column vectors of the poses of the edges
        %einfs:  3d matrix containing the information matrices of the edges
        %	 einfs(:,:,k) refers to the information matrix of the k-th edge.
        %n:	 number of iterations
        %newmeans: matrix containing the column vectors of the updated vertices positions
        
        function g2 = optimize(pg, varargin)
            
            opt.iterations = 10;
            opt.animate = false;
            opt.retain = false;
            
            opt = tb_optparse(opt, varargin);
            
            g2 = PGraph(pg.graph);  % deep copy
            
            eprev = Inf;
            for i=1:opt.iterations
                if opt.animate
                    if ~opt.retain
                        clf
                    end
                    g2.plot();
                    pause(0.5)
                end
                
                [vmeans,energy] = linearize_and_solve(g2);
                g2.setcoord(vmeans);
                
                
                if energy >= eprev
                    break;
                end
                eprev = energy;
            end;
            
            pg.graph = g2;
        end
    end % methods
end % classdef




%computes the taylor expansion of the error function of the k_th edge
%vmeans: vertices positions
%eids:   edge ids
%emeans: edge means
%k:	 edge number
%e:	 e_k(x)
%A:	 d e_k(x) / d(x_i)
%B:	 d e_k(x) / d(x_j)
%function [e, A, B]=linear_factors(vmeans, eids, emeans, k)
function [e, A, B]=linear_factors(g, edge)
    %extract the ids of the vertices connected by the kth edge
    % 	id_i=eids(1,k);
    % 	id_j=eids(2,k);
    %extract the poses of the vertices and the mean of the edge
    %     v_i=vmeans(:,id_i);
    %     v_j=vmeans(:,id_j);
    %     z_ij=emeans(:,k);
    
    v = g.vertices(edge);
    v_i = g.coord(v(1));
    v_j = g.coord(v(2));
    z_ij = g.edata(edge).mean;
    
    %compute the homoeneous transforms of the previous solutions
    zt_ij=v2t(z_ij);
    vt_i=v2t(v_i);
    vt_j=v2t(v_j);
    
    %compute the displacement between x_i and x_j
    
    
    f_ij=(inv(vt_i)*vt_j);
    
    %this below is too long to explain, to understand it derive it by hand
    theta_i=v_i(3);
    ti=v_i(1:2,1);
    tj=v_j(1:2,1);
    dt_ij=tj-ti;
    
    si=sin(theta_i);
    ci=cos(theta_i);
    
    A= [-ci, -si, [-si, ci]*dt_ij; si, -ci, [-ci, -si]*dt_ij; 0, 0, -1 ];
    B =[  ci, si, 0           ; -si, ci, 0            ; 0, 0, 1 ];
    
    ztinv=inv(zt_ij);
    e=t2v(ztinv*f_ij);
    ztinv(1:2,3) = 0;
    A=ztinv*A;
    B=ztinv*B;
    
    % 	%compute the homogeneous transforms of the previous solutions
    % 	zt_ij=v2t(z_ij);
    % 	vt_i=v2t(v_i);
    % 	vt_j=v2t(v_j);
    % %     zt_ij = SE2(z_ij);
    % % 	vt_i = SE2(v_i);
    % % 	vt_j = SE2(v_j);
    %
    % 	%compute the displacement between x_i and x_j
    % 	%f_ij=(inverse(vt_i)*vt_j);
    % 	f_ij = vt_i.inv * vt_j;
    %
    % 	%this below is too long to explain, to understand it derive it by hand
    %       	theta_i=v_i(3);
    % 	ti=v_i(1:2);
    % 	tj=v_j(1:2);
    %       	dt_ij=tj-ti;
    %
    % 	si=sin(theta_i);
    % 	ci=cos(theta_i);
    %
    % 	A= [-ci, -si, [-si, ci]*dt_ij; si, -ci, [-ci, -si]*dt_ij; 0, 0, -1 ];
    % 	B =[  ci, si, 0           ; -si, ci, 0            ; 0, 0, 1 ];
    %
    % 	ztinv = inv(zt_ij);
    % 	e = xyt(ztinv*f_ij);
    % 	ztinv.t = 0;
    % 	A = ztinv*A;
    % 	B = ztinv*B;
end



%linearizes and solves one time the ls-slam problem specified by the input
%vmeans:   vertices positions at the linearization point
%eids:     edge ids
%emeans:   edge means
%einfs:    edge information matrices
%newmeans: new solution computed from the initial guess in vmeans
function [newmeans,energy] = linearize_and_solve(g)
    tic
    fprintf('solving');
    
    % H and b are respectively the system matrix and the system vector
    H=zeros(g.n*3,g.n*3);
    b=zeros(g.n*3,1);
    % this loop constructs the global system by accumulating in H and b the contributions
    % of all edges (see lecture)
    %for k=1:size(eids,2)
    fprintf('.');
    
    etotal = 0;
    for edge = 1:g.ne
        
        [e, A, B]=linear_factors(g, edge);
        omega = g.edata(edge).info;
        %compute the blocks of H^k
        
        % not quite sure whey SE3 is being transposed, what does that mean?
        b_i = -A'*omega*e;
        b_j = -B'*omega*e;
        H_ii = A'*omega*A;
        H_ij = A'*omega*B;
        H_jj = B'*omega*B;
        
        v = g.vertices(edge);
        id_i = v(1); id_j = v(2);
        %accumulate the blocks in H and b
        H((id_i-1)*3+1:id_i*3,(id_i-1)*3+1:id_i*3) = H((id_i-1)*3+1:id_i*3,(id_i-1)*3+1:id_i*3) + H_ii;
        H((id_j-1)*3+1:id_j*3,(id_j-1)*3+1:id_j*3) = H((id_j-1)*3+1:id_j*3,(id_j-1)*3+1:id_j*3) + H_jj;
        H((id_i-1)*3+1:id_i*3,(id_j-1)*3+1:id_j*3) = H((id_i-1)*3+1:id_i*3,(id_j-1)*3+1:id_j*3) + H_ij;
        H((id_j-1)*3+1:id_j*3,(id_i-1)*3+1:id_i*3) = H((id_j-1)*3+1:id_j*3,(id_i-1)*3+1:id_i*3) + H_ij';
        
        b((id_i-1)*3+1:id_i*3,1) = b((id_i-1)*3+1:id_i*3,1) + b_i;
        b((id_j-1)*3+1:id_j*3,1) = b((id_j-1)*3+1:id_j*3,1) + b_j;
        
        %NOTE on Matlab compatibility: note that we use the += operator which is octave specific
        %using H=H+.... results in a tremendous overhead since the matrix would be entirely copied every time
        %and the matrix is huge
        etotal = etotal + e'*e;
    end;
    fprintf('.');
    
    %note that the system (H b) is obtained only from
    %relative constraints. H is not full rank.
    %we solve the problem by anchoring the position of
    %the the first vertex.
    %this can be expressed by adding the equation
    %  deltax(1:3,1)=0;
    %which is equivalent to the following
    H(1:3,1:3) = H(1:3,1:3) + eye(3);
    
    SH=sparse(H);
    fprintf('.');
    deltax=SH\b;
    fprintf('.');
    
    %split the increments in nice 3x1 vectors and sum them up to the original matrix
    newmeans = g.coord()+reshape(deltax,3,g.n);
    
    %normalize the angles between -PI and PI
    for (i=1:size(newmeans,2))
        s=sin(newmeans(3,i));
        c=cos(newmeans(3,i));
        newmeans(3,i)=atan2(s,c);
    end
    dt = toc;
    fprintf('done in %.2g sec.  Total cost %g \n', dt, etotal);
    if nargout > 1
        energy = etotal;
    end
end

%   This source code is part of the graph optimization package
%   deveoped for the lectures of robotics2 at the University of Freiburg.
%
%     Copyright (c) 2007 Giorgio Grisetti, Gian Diego Tipaldi
%
%   It is licences under the Common Creative License,
%   Attribution-NonCommercial-ShareAlike 3.0
%
%   You are free:
%     - to Share - to copy, distribute and transmit the work
%     - to Remix - to adapt the work
%
%   Under the following conditions:
%
%     - Attribution. You must attribute the work in the manner specified
%       by the author or licensor (but not in any way that suggests that
%       they endorse you or your use of the work).
%
%     - Noncommercial. You may not use this work for commercial purposes.
%
%     - Share Alike. If you alter, transform, or build upon this work,
%       you may distribute the resulting work only under the same or
%       similar license to this one.
%
%   Any of the above conditions can be waived if you get permission
%   from the copyright holder.  Nothing in this license impairs or
%   restricts the author's moral rights.
%
%   This software is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied
%   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
%   PURPOSE.

%computes the homogeneous transform matrix A of the pose vector v
function A=v2t(v)
    c=cos(v(3));
    s=sin(v(3));
    A=[c, -s, v(1) ;
        s,  c, v(2) ;
        0   0  1  ];
end

%computes the pose vector v from an homogeneous transform A
function v=t2v(A)
    v(1:2, 1)=A(1:2,3);
    v(3,1)=atan2(A(2,1),A(1,1));
end
