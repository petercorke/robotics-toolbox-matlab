%% CODEGENERATOR Class for code generation from symboic SerialLink objects.
%
% Objects of the codeGenerator class enable the automated robot specific
% model code generation for SerialLink robot objects.
%
% Currently the generation supports:
% - mat-files with symbolic robot specific model expressions
% - m-functions with symbolic robot specific model code
% - real-time capable robot specific Simulink blocks
%
% Constructor::
%   cGen = codeGenerator(rob)
%   cGen = codeGenerator(rob,'optionSet')
%   cGen = codeGenerator(...,'par1', val1, 'par2', val2,... )
%
%       'optionSet' || verbose || saveResult ||    logFile     || genMFun    || genSLBlock    ||
%       -------------++---------++------------++----------------++------------++---------------++
%       'default'   ||    1    ||     1      ||      ''        ||     1      ||      1        ||
%       'debug'     ||    1    ||     1      || 'robModel.log' ||     1      ||      1        ||
%       'silent'    ||    0    ||     1      ||      ''        ||     1      ||      1        ||
%       'disk'      ||    1    ||     1      ||      ''        ||     0      ||      0        ||
%       'workspace' ||    1    ||     0      ||      ''        ||     0      ||      0        ||
%       'mfun     ' ||    1    ||     1      ||      ''        ||     1      ||      0        ||
%       'slblock  ' ||    1    ||     1      ||      ''        ||     0      ||      1        ||
%
%   The option sets can be passed as an optional parameter to the symbolic
%   model creation functions. If 'optionSet' is ommitted, then 'default' is used.
%
% Methods::
%
%  getrobfname   get robot name and remove any blanks
%  savesym       save symbolic expression to disk
% 
%  genfkine      generate forward kinematics code
%  genjacobian   generate jacobian code
%  genfdyn       generate forward dynamics code
%  geninvdyn     generate forward dynamics code
%  geneverything generate code for all of the above
%
% Properties (read/write)::
%
%  basepath       basic working directory of the code generator
%  robjpath;      subdirectory for specialized matlab functions
%  sympath;       subdirectory for symbolic expressions
%  slib;          filename of the simulink library
%  slibpath;      subdirectory for the simulink library
%  verbose;       print code generation progress on console (bool)
%  saveresult;    save symbolic expressions to .mat-files (bool)
%  logfile;       print modeling progress to specified text file (string)
%  genmfun;       generate executable M-functions (bool)
%  genslblock;    generate Embedded Matlab Function blocks (bool)
%
% Object properties (read only)::
%  rob            SerialLink object to generate code for (1x1).
%
%
%  Authors::
%        Jörn Malzahn
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de
%
% See also SerialLink, Link.

classdef codeGenerator
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
        function CGen = codeGenerator(rob,varargin)
            if ~isa(rob,'SerialLink')
                error('codeGenerator:wrongConstructorInput','The input variable %s must be a SerialLink object.',inputname(1));
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
    
        end
        
        function robName = getrobfname(CGen)
            robName = CGen.rob.name;
            blanks = isspace(robName)==1;
            robName(blanks)= [];
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
            CGen.genjacobian;
            CGen.genfdyn;
            CGen.geninvdyn;
        end
        
    end
    
end

