function v = mvar(fmt, varargin)

    if isempty(strfind(fmt, '('))
        % not a function
    v = sym( sprintf(fmt, varargin{:}), 'real' );
    else
            v = sym( sprintf(fmt, varargin{:}) );

    end
