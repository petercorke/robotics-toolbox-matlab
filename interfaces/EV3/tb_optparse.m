%OPTPARSE Standard option parser for Toolbox functions
%
% [OPTOUT,ARGS] = TB_OPTPARSE(OPT, ARGLIST) is a generalized option parser for
% Toolbox functions.  It supports options that have an assigned value, boolean 
% or enumeration types (string or int).
%
% The software pattern is:
%
%       function(a, b, c, varargin)
%       opt.foo = true;
%       opt.bar = false;
%       opt.blah = [];
%       opt.choose = {'this', 'that', 'other'};
%       opt.select = {'#no', '#yes'};
%       opt = tb_optparse(opt, varargin);
%
% Optional arguments to the function behave as follows:
%   'foo'           sets opt.foo <- true
%   'nobar'         sets opt.foo <- false
%   'blah', 3       sets opt.blah <- 3
%   'blah', {x,y}   sets opt.blah <- {x,y}
%   'that'          sets opt.choose <- 'that'
%   'yes'           sets opt.select <- 2 (the second element)
%
% and can be given in any combination.
%
% If neither of 'this', 'that' or 'other' are specified then opt.choose <- 'this'.
% If neither of 'no' or 'yes' are specified then opt.select <- 1.
%
% Note:
% - that the enumerator names must be distinct from the field names.
% - that only one value can be assigned to a field, if multiple values
%    are required they must be converted to a cell array.
%
% The allowable options are specified by the names of the fields in the
% structure opt.  By default if an option is given that is not a field of 
% opt an error is declared.  
%
% Sometimes it is useful to collect the unassigned options and this can be 
% achieved using a second output argument
%           [opt,arglist] = tb_optparse(opt, varargin);
% which is a cell array of all unassigned arguments in the order given in
% varargin.
%
% The return structure is automatically populated with fields: verbose and
% debug.  The following options are automatically parsed:
%   'verbose'           sets opt.verbose <- true
%   'verbose=2'         sets opt.verbose <- 2 (very verbose)
%   'verbose=3'         sets opt.verbose <- 3 (extremeley verbose)
%   'verbose=4'         sets opt.verbose <- 4 (ridiculously verbose)
%   'debug', N          sets opt.debug <- N
%   'setopt', S         sets opt <- S
%   'showopt'           displays opt and arglist

function [opt,others] = tb_optparse(in, argv)

    arglist = {};

    argc = 1;
    opt = in;
    try
        opt.verbose = false;
        opt.debug = 0;
    end

    showopt = false;

    while argc <= length(argv)
        option = argv{argc};
        assigned = false;
        
        if isstr(option)

            switch option
            % look for hardwired options
            case 'verbose'
                opt.verbose = true;
                assigned = true;
            case 'verbose=2'
                opt.verbose = 2;
                assigned = true;
            case 'verbose=3'
                opt.verbose = 3;
                assigned = true;
            case 'verbose=4'
                opt.verbose = 4;
                assigned = true;
            case 'debug'
                opt.debug = argv{argc+1};
                argc = argc+1;
                assigned = true;
            case 'setopt'
                new = argv{argc+1};
                argc = argc+1;
                assigned = true;


                % copy matching field names from new opt struct to current one
                for f=fieldnames(new)'
                    if isfield(opt, f{1})
                        opt = setfield(opt, f{1}, getfield(new, f{1}));
                    end
                end
            case 'showopt'
                showopt = true;
                assigned = true;

            otherwise
                % does the option match a field in the opt structure?
                if isfield(opt, option)
                    val = getfield(opt, option);
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt = setfield(opt, option, true);
                    else
                        % otherwise grab its value from the next arg
                        opt = setfield(opt, option, argv{argc+1});
                        argc = argc+1;
                    end
                    assigned = true;
                elseif length(option)>2 && strcmp(option(1:2), 'no') && isfield(opt, option(3:end))
                    val = getfield(opt, option(3:end));
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt = setfield(opt, option(3:end), false);
                        assigned = true;
                    end
                else
                    % the option doesnt match a field name
                    for field=fieldnames(opt)'
                        val = getfield(opt, field{1});
                        if iscell(val)
                            for i=1:length(val)
                                if isempty(val{i})
                                    continue;
                                end
                                if strcmp(option, val{i})
                                    opt = setfield(opt, field{1}, option);
                                    assigned = true;
                                    break;
                                elseif val{i}(1) == '#' && strcmp(option, val{i}(2:end))
                                    opt = setfield(opt, field{1}, i);
                                    assigned = true;
                                    break;
                                end
                            end
                            if assigned
                                break;
                            end
                        end
                    end


                end
            end % switch
        end
        if ~assigned
            % non matching options are collected
            if nargout == 2
                arglist = [arglist argv(argc)];
            else
                if isstr(argv{argc})
                    error(['unknown options: ' argv{argc}]);
                end
            end
        end
        
        argc = argc + 1;
    end % while

    % if enumerator value not assigned, set the default value
    for field=fieldnames(in)'
        if iscell(getfield(in, field{1})) && iscell(getfield(opt, field{1}))
            val = getfield(opt, field{1});
            if isempty(val{1})
                opt = setfield(opt, field{1}, val{1});
            elseif val{1}(1) == '#'
                opt = setfield(opt, field{1}, 1);
            else
                opt = setfield(opt, field{1}, val{1});
            end
        end
    end
                        
    if showopt
        fprintf('Options:\n');
        opt
        arglist
    end

    if nargout == 2
        others = arglist;
    end
