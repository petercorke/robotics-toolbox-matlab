%CODEGENERATOR Class for code generation
%
% Objects of the CodeGenerator class automatcally generate robot specific
% code, as either M-functions or real-time capable SerialLink blocks.
%
% The various methods return symbolic expressions for robot kinematic and
% dynamic functions, and optionally support side effects such as:
% - M-functions with symbolic robot specific model code
% - real-time capable robot specific Simulink blocks
% - mat-files with symbolic robot specific model expressions
%
% Example::
%
%        % load robot model
%        mdl_twolink
%
%        cg = CodeGenerator(twolink);
%        cg.geneverything();
%
%        % a new class has been automatically generated in the robot directory.
%        addpath robot
%
%        tl = @robot();
%        % this class is a subclass of SerialLink, and thus polymorphic with
%        % SerialLink but its methods have been overloaded with robot-specific code,
%        % for example
%        T = tl.fkine([0.2 0.3]);
%        % uses concise symbolic expressions rather than the generalized A-matrix
%        % approach
%
%        % The Simulink block library containing robot-specific blocks can be
%        % opened by
%        open robot/robotslib.slx
%        % and the blocks dragged into your own models.
%
% Methods::
%
%  gencoriolis     generate Coriolis/centripetal code
%  genfdyn         generate forward dynamics code
%  genfkine        generate forward kinematics code
%  genfriction     generate joint frictionc code
%  gengravload     generarte gravity load code
%  geninertia      general inertia matrix code
%  geninvdyn       generate forward dynamics code
%  genjacobian     generate Jacobian code
%  geneverything   generate code for all of the above
%
% Properties (read/write)::
%
%  basepath       basic working directory of the code generator
%  robjpath       subdirectory for specialized MATLAB functions
%  sympath        subdirectory for symbolic expressions
%  slib           filename of the Simulink library
%  slibpath       subdirectory for the Simulink library
%  verbose        print code generation progress on console (logical)
%  saveresult     save symbolic expressions to .mat-files (logical)
%  logfile        print modeling progress to specified text file (string)
%  genmfun        generate executable M-functions (logical)
%  genslblock     generate Embedded MATLAB Function blocks (logical)
%
% Object properties (read only)::
%  rob            SerialLink object to generate code for (1x1).
%
% Notes::
%  - Requires the MATLAB Symbolic Toolbox
%  - For robots with > 3 joints the symbolic expressions are massively
%    complex, they are slow and you may run out of memory.
%  - As much as possible the symbolic calculations are down row-wise to
%    reduce the computation/memory burden.
%
% Author::
%  Joern Malzahn
%  2012 RST, Technische Universitaet Dortmund, Germany.
%  http://www.rst.e-technik.tu-dortmund.de
%
% See also SerialLink, Link.

% Copyright (C) 1993-2012, by Peter I. Corke
% Copyright (C) 2012-2013, by Joern Malzahn
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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
%
% The code generation module emerged during the work on a project funded by
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully 
% acknowledge the financial support.

classdef CodeGenerator
    properties (SetAccess = private)
        rob
    end
    properties
        basepath;
        robjpath;
        sympath;
        slib;
        slibpath;
        verbose;
        saveresult;
        logfile;
        genmfun;
        genslblock;
    end
    properties (GetAccess = private)
        debug;        % just appears because of tb_optparse, so hide it from the user
    end
    methods

        function CGen = CodeGenerator(rob,varargin)
        %CodeGenerator.CodeGenerator Construct a code generator object
        %
        % cGen = CodeGenerator(ROB) is a code generator object for the SerialLink
        % object ROB.
        %
        % cGen = CodeGenerator(ROB, OPTIONS) as above but with options described below.
        %
        % Options::
        %
        % The following option sets can be passed as an optional parameter:
        %
        %  'default'     set the options: verbose, saveResult, genMFun, genSLBlock
        %  'debug'       set the options: verbose, saveResult, genMFun, genSLBlock 
        %                and create a logfile named 'robModel.log' in the working directory
        %  'silent'      set the options: saveResult, genMFun, genSLBlock
        %  'disk'        set the options: verbose, saveResult
        %  'workspace'   set the option: verbose; just outputs symbolic expressions to workspace
        %  'mfun'        set the options: verbose, saveResult, genMFun
        %  'slblock'     set the options: verbose, saveResult, genSLBlock
        %
        % If 'optionSet' is ommitted, then 'default' is used. The options control the code generation and user information: 
        %
        %  'verbose'           write code generation progress to command window
        %  'saveResult         save results to hard disk (always enabled, when genMFun and genSLBlock are set)
        %  'logFile',logfile   write code generation progress to specified logfile
        %  'genMFun'           generate robot specific m-functions
        %  'genSLBlock'        generate real-time capable robot specific Simulink blocks
        % 
        %  Any option may also be modified individually as optional parameter value pairs.
        %
        % Author::
        %  Joern Malzahn
        %  2012 RST, Technische Universitaet Dortmund, Germany.
        %  http://www.rst.e-technik.tu-dortmund.de

            if ~isa(rob,'SerialLink')
                error('CodeGenerator:wrongConstructorInput','The input variable %s must be a SerialLink object.',inputname(1));
            end
            
            if ~issym(rob)
                CGen.rob = rob.sym;
            else
                CGen.rob = rob;
            end
            
            % defaults
            CGen.basepath = fullfile(pwd,CGen.getrobfname);
            CGen.robjpath = fullfile(CGen.basepath,['@',CGen.getrobfname]);
            CGen.sympath = fullfile(CGen.basepath,'symbolicexpressions');
            CGen.slib = [CGen.getrobfname,'slib'];
            CGen.slibpath = fullfile(CGen.basepath,CGen.slib);
            CGen.verbose = false;
            CGen.saveresult = false;
            CGen.logfile = '';
            CGen.genmfun = false;
            CGen.genslblock = false;
            CGen.debug = false;
            
            
            if nargin < 2
                varargin{1} = 'default';
            end
            
            % Determine code generation option set
            switch varargin{1}
                case 'default'
                    CGen = tb_optparse(CGen,...
                        [{'verbose','saveresult','genmfun','genslblock'},varargin(2:end)]);
                case 'debug'
                    CGen = tb_optparse(CGen,...
                        [{'verbose','saveresult','genmfun','genslblock','logfile','robModel.log'},varargin(2:end)]);
                case 'silent'
                    CGen = tb_optparse(CGen,...
                        [{'saveresult','genmfun','genslblock'},varargin(2:end)]);
                case 'disk'
                    CGen = tb_optparse(CGen,...
                        [{'verbose','saveresult'},varargin(2:end)]);
                case 'workspace'
                    CGen = tb_optparse(CGen,...
                        [{'verbose'},varargin(2:end)]);
                case 'mfun'
                    CGen = tb_optparse(CGen,...
                        [{'verbose','saveresult','genmfun'},varargin(2:end)]);
                case 'slblock'
                    CGen = tb_optparse(CGen,...
                        [{'verbose','saveresult','genslblock'},varargin(2:end)]);
                otherwise
                    CGen = tb_optparse(CGen,varargin);
            end
            
            if any([CGen.genmfun, CGen.genslblock])
                CGen.saveresult = true;
            end
            
            if ~isempty(CGen.logfile)
                logfid = fopen(CGen.logfile,'w+'); % open or create file, discard existing contents
                fclose(logfid);
            end
            CGen.logmsg([datestr(now),' +++++++++++++++++++++++++++++++++++\n']);
            CGen.logmsg([datestr(now),'\tLog for ',CGen.getrobfname,'\n']);
            CGen.logmsg([datestr(now),' +++++++++++++++++++++++++++++++++++\n']);
            
        end
        
        function robName = getrobfname(CGen)
            robName = CGen.rob.name;
            blanks = isspace(robName)==1;
            robName(blanks)= [];
            robName(robName=='/') = [];
        end

        function savesym(CGen,sym2save, symname, fname)
            if ~exist(CGen.sympath,'dir')
                mkdir(CGen.sympath)
            end
            
            eval([symname,'= sym2save;']);
            save(fullfile(CGen.sympath,fname),symname);
        end

        function [] = geneverything(CGen)
            [t,allT] = CGen.genfkine;
            [J0,Jn] = CGen.genjacobian;
            [G] = CGen.gengravload;
            [I] = CGen.geninertia;
            [C] = CGen.gencoriolis;
            [F] = CGen.genfriction;
            [Iqdd] = CGen.genfdyn;
            [tau] = CGen.geninvdyn;
        end

        function CGen = set.genmfun(CGen,value)
            CGen.genmfun = value;
            if value == true
                CGen.saveresult = value;
            end
            
            if ~exist(fullfile(CGen.robjpath,CGen.getrobfname),'file')
                CGen.logmsg([datestr(now),'\tCreating ',CGen.getrobfname,' m-constructor ']);
                CGen.createmconstructor;
                CGen.logmsg('\t%s\n',' done!');
            end
        end

        function CGen = set.genslblock(CGen,value)
            CGen.genslblock = value;
            if value == true
                CGen.saveresult = true;
            end
        end
        function [] = addpath(CGen)
        %CodeGenerator.addpath Adds generated code to search path
        %
        % cGen.addpath() adds the generated m-functions and block library to the 
        % MATLAB function search path.
        %
        % Author::
        %  Joern Malzahn
        %  2012 RST, Technische Universitaet Dortmund, Germany.
        %  http://www.rst.e-technik.tu-dortmund.de
        %
        % See also addpath.
        addpath(CGen.basepath);
        end
        
        function [] = rmpath(CGen)
        %CodeGenerator.rmpath Removes generated code from search path
        %
        % cGen.rmpath() removes generated m-functions and block library from the 
        % MATLAB function search path.
        %
        % Author::
        %  Joern Malzahn
        %  2012 RST, Technische Universitaet Dortmund, Germany.
        %  http://www.rst.e-technik.tu-dortmund.de
        %
        % See also rmpath.
        rmpath(CGen.basepath);
        end
        
        function [] = purge(CGen,varargin)
        %CodeGenerator.purge  Cleanup generated files
        %
        % cGen.purge() deletes all generated files, first displays a question dialog to 
        % make sure the user really wants to delete all generated files.
        %
        % cGen.purge(1) as above but skips the question dialog.
        %
        % Author::
        %  Joern Malzahn
        %  2012 RST, Technische Universitaet Dortmund, Germany.
        %  http://www.rst.e-technik.tu-dortmund.de
            dopurge = 0;
            
            if exist(CGen.basepath,'dir')
                if nargin > 1
                    dopurge = varargin{1}
                else
                    qstn = ['Do you really want to delete ',CGen.basepath, ' and all of it''s contents?'];
                    tit = ['Purge: ',CGen.getrobfname];
                    str1 = 'Yes';
                    str2 = 'No';
                    button = questdlg(qstn,tit,str1,str2,str1)
                    dopurge = strcmp(button,str1);
                end
            end
            
            if dopurge
                bdclose all
                
                warning off;
                CGen.rmpath;
                warning on;
                
                rmdir(CGen.basepath,'s')
                if ~isempty(CGen.logfile) && exist(CGen.logfile,'file')
                    delete(CGen.logfile);
                end
            end
        end
    end
end
