%DYNAMICSIDENTIFIER Class for robot dynamics identification
%
% Objects of the DynamicsIdentifier class produces the parameter
% linear robot dynamics equations in regression form.
%
% The various methods ...
%
% Example::
%   mdl_twolink;
%   did = DynamicsIdentifier(twolink)
%   [YB, XB] = did.getbaseyx
%
% Methods::
%
%
% Properties (read/write)::
%
%
% Object properties (read only)::
%
% Notes::
%   This module is new to RTB. Be aware of possible substantial
%    changes in terms of function names and usage in the near future.
%
% Author::
%  Joern Malzahn (joern.malzahn@tu-dortmund.de)
%  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
%  www.iit.it
%
% See also CodeGenerator, SerialLink.

% Copyright (C) 1993-2012, by Peter I. Corke
% Copyright (C) 2012-2015, by Joern Malzahn
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
% along with RTB. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
%
% The code generation module originally emerged during the work on a project 
% funded by the German Research Foundation (DFG, BE1569/7-1). The authors 
% gratefully acknowledge the financial support.


classdef DynamicsIdentifier
    
    properties (SetAccess = private)
        rob
        robn
    end
    
    properties
        verbose
        saveresult
    end
    
    methods
        function DId = DynamicsIdentifier(rob,varargin)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            fprintf(2,'Note: this module is new to RTB. Be aware of possible substantial\n');
            fprintf(2,'changes in terms of function names and usage in the near future.\n');
            DId.rob = DynamicsIdentifier.preparerobot(rob.nofriction);
            DId.robn = rob.nofriction;
        end
        
        function [Y, x, symExpr] = regressor_gravload(DId)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            q = DId.rob.gencoords;
            
            symExpr = DId.rob.gravload(q).';
            
            [Y, x] = DId.getminyx_gravload(symExpr);
        end
        
        
        function [Y, x, symExpr] = regressor_inertia(DId)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            [q] = DId.rob.gencoords;
            
            symExpr = DId.rob.inertia(q);
            
            [Y, x] = DId.getminyx_inertia(symExpr);
        end
        
        function [Y, x, symExpr] = regressor_coriolis(DId)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            [q, qd] = DId.rob.gencoords;
            
            symExpr = DId.rob.coriolis(q,qd);
            
            [Y, x] = DId.getminyx_coriolis(symExpr);
        end
        % ----
        
        function [YB, XB] = getbaseyx(DId,varargin)
            % DynamicsIdentifier.getbaseyx Derives the symbolic vector of base
            % paramters along with the coresponding symbolic regressor matrix.
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
           doOptimizeRegressor = 0;
            if nargin > 1
                doOptimizeRegressor = varargin{1};
            end
            
            [Yg,xg] = DId.regressor_gravload;
            
            [Yi,xi] = DId.regressor_inertia;
            
            [Yc,xc] = DId.regressor_coriolis;
            
            fullY = [Yi,Yc,Yg];
            fullX = [xi; xc; xg];

            
            [stdY, stdX] = DynamicsIdentifier.minimalyx(fullY,fullX);
            
            
            [YB,XB] = DynamicsIdentifier.std2baseyx(stdY, stdX, doOptimizeRegressor);
            
        end
        
    end
    
    methods(Static)
        
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % higher level functions for regressor generation
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [Y, x] = getminyx_ICG(iExpr, cExpr, gExpr)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            [Yi,xi] = DynamicsIdentifyer.getminyx_inertia(iExpr);
            
            [Yc,xc] = DynamicsIdentifyer.getminyx_coriolis(cExpr);
            
            [Yg,xg] = DynamicsIdentifyer.getminyx_gravload(gExpr);
            
            fullY = [Yi,Yc,Yg];
            fullX = [xi; xc; xg];
            
            [Y, x] = minimalyx(fullY, fullX);
        end
        
        function [Y, x] = getminyx_inertia(inExpr)
            
            [largeY,largeX] = DynamicsIdentifier.splityx_inertia(inExpr);
            [Y,x] = DynamicsIdentifier.minimalyx(largeY,largeX);
        end
        
        function [Y, x] = getminyx_coriolis(inExpr)
            [largeY,largeX] = DynamicsIdentifier.splityx_coriolis(inExpr);
            [Y,x] = DynamicsIdentifier.minimalyx(largeY,largeX);
        end
        
        function [Y, x] = getminyx_gravload(inExpr)
            [largeY,largeX] = DynamicsIdentifier.splityx_gravload(inExpr);
            [Y,x] = DynamicsIdentifier.minimalyx(largeY,largeX);
        end

        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % lower level functions for regressor generation
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [largeY,largeX] = splityx_inertia(inExpr)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            nRows = size(inExpr,1);
            nCols = size(inExpr,2);
            
            qddString =  sprintf('qdd%d ', 1:nRows);
            eval(['syms ',  qddString]);
            eval(['qdd = [', qddString,'].'';']);
            
            largeX = [];
            largeY = [];
            
            for iRow = 1:nRows
                
                rowY = [];
                rowX = [];
                
                for iCol = 1:nCols
                    curEntry = (expand(inExpr(iRow,iCol)));
                    
                    curString = char(curEntry);
                    
                    [locSigns,allSummands] = regexp(curString,'-|+','match','split');
                    if numel(locSigns) < numel(allSummands)
                        locSigns = ['+',locSigns];
                    end
                    
                    
                    nSummands = numel(allSummands);
                    colR = cell(nSummands,1);
                    colX = cell(nSummands,1);
                    for iSummand = 1:nSummands
                        curSummand = allSummands(iSummand);
                        
                        % Regular expression: match sine or cosine followed by any character
                        % [except a closing bracket] written in brackets followed by possibly
                        % one or none multiplication character \*. This pattern may repeat zero
                        % or more times. The exception for the bracket within the brackets is
                        % necessary to prevent double brackets to be matched.
                        regExp1 = '(sin|cos)'; % sine or cosine followed by
                        regExp2 = '\(';        % a left round bracket followed by
                        regExp3 = '[^\)]*';    % any number of characters except a right round bracket (this is the argument to the sine/cosine), followed by
                        regExp4 = '(\)){1}';   % exactly one right bracket (closes the argument list), followed by
                        regExp5 = '(\^\d+){0,1}';   % possibly a power to the trigonometric function with multiple, followed by
                        regExp6 = '\*{0,1}';       % possibly one power multiplication character, followed by
                        
                        regExpStr = ['(',regExp1,regExp2,regExp3,regExp4,regExp5,regExp6,')*'];
                        
                        % extract regressor entry
                        [~, matchstring] = regexp(curSummand,regExpStr,'split','match');
                        if isempty(matchstring{1})
                            matchstring{1} = '1';
                        end
                        colR(iSummand) = strcat(locSigns(iSummand),'(', matchstring{1},')');
                        
                        % extract parameter
                        param = regexprep(curSummand, regExpStr, '1');
                        colX{iSummand} = param{1};
                        
                    end
                    
                    % Insert found regressors and parameters in large parameter vector and
                    % large regressor matrix.
                    rowX = [rowX; colX(:)];
                    rowY = [rowY; colR(:)*qdd(iCol)];
                end
                
                if isempty(rowY)
                    rowY{1} = '0';
                    rowX{1} = '0';
                end
                
                % First extend the large regressor matrix
                largeY = [largeY, sym(zeros(size(largeY,1),numel(rowY)));
                    sym(zeros(1,size(largeY,2))), sym(rowY(:).')];
                largeX = [largeX; rowX(:)];
            end
            
            % remove zero parameters
            zIdx = strcmp(largeX,'0');
            zIdx = find(zIdx);
            largeX(zIdx) = [];
            largeY(:,zIdx) = [];
            
            % Check
            test = simplify(largeY*largeX-inExpr*qdd(:));
            if any(test ~= zeros(size(largeY,1),1))
                warning('Warning DynamicsIdentifier.splityx_inertia has made a mistake!')
            end
        end
        
        function [largeY, largeX] = splityx_coriolis(inExpr)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            nRows = size(inExpr,1);
            nCols = size(inExpr,2);
            
            qdString =  sprintf('qd%d ', 1:nRows);
            eval(['syms ',  qdString]);
            eval(['qd = [', qdString,'].'';']);
            
            largeX = [];
            largeY = sym([]);
            
            for iRow = 1:nRows
                
                rowY = [];
                rowX = [];
                
                for iCol = 1:nCols
                    curEntry = (expand(inExpr(iRow,iCol)));
                    
                    curString = char(curEntry);
                    
                    [locSigns,allSummands] = regexp(curString,'-|+','match','split');
                    if isempty(allSummands{1})
                        allSummands = allSummands(2:end);
                    end
                    if (numel(locSigns) < numel(allSummands))
                        locSigns = ['+',locSigns];
                    end
                    
                    
                    nSummands = numel(allSummands);
                    if nSummands == 1
                        if strcmp(curString,'0')
                            continue;
                        end
                    end
                    
                    colY = cell(nSummands,1);
                    colX = cell(nSummands,1);
                    for iSummand = 1:nSummands
                        
                        curSummand = allSummands(iSummand);
                        
                        
                        % Regular expression: match sine or cosine followed by any character
                        % [except a closing bracket] written in brackets followed by possibly
                        % one or none multiplication character \*. This pattern may repeat zero
                        % or more times. The exception for the bracket within the brackets is
                        % necessary to prevent double brackets to be matched.
                        % regExpStr = '((sin|cos)\([^\)]*(\)){1}\*{0,1})*';
                        
                        % Match and split string if you find an expression starting with
                        regExp0 = '(qd\d)?';    % eventually a joint value derivative
                        regExp1 = '(sin|cos)'; % sine or cosine followed by
                        regExp2 = '\(';        % a left round bracket followed by
                        regExp3 = '[^\)]*';    % any number of characters except a right round bracket (this is the argument to the sine/cosine) followed by
                        regExp4 = '(\)){1}';   % exactly one right bracket (closes the argument list), followed by
                        regExp5 = '(\^\d+){0,1}';   % possibly a power to the trigonometric function with multiple, followed by
                        regExp6 = '\*?';       % possibly one power multiplication character, followed by
                        %         regExpStr = '((sin|cos)\([^\)]*(\)){1}(\x5E\d)?\*?)*';
                        %             regExpStr = ['(',regExp0,regExp1,regExp2,regExp3,regExp4,regExp5,regExp6,')*'];
                        regExpStr = ['(',regExp1,regExp2,regExp3,regExp4,regExp5,regExp6,')*'];
                        
                        % extract regressor entry
                        [~, matchstring] = regexp(curSummand,regExpStr,'split','match'); % look for trigonometric functions and their powers
                        [matchstringQD] = regexp(curSummand,regExp0,'match');       % look for generalized velocity as a separate factor
                        
                        if isempty(matchstring{1})
                            matchstring{1} = '1';
                        end
                        if isempty(matchstringQD{1})
                            matchstringQD{1} = '1';
                        end
                        
                        colY(iSummand) = strcat(locSigns(iSummand),'(', matchstring{1},'*',matchstringQD{1},')');
                        
                        % extract parameter
                        param = regexprep(curSummand, regExpStr, '1');
                        param = regexprep(param, regExp0, '1');
                        colX{iSummand} = param{1};
                        
                        
                        
                    end
                    
                    % Insert found regressors and parameters in large parameter vector and
                    % large regressor matrix.
                    rowX = [rowX; colX(:)];
                    rowY = [rowY; colY(:)*qd(iCol)];
                end
                
                if isempty(rowY)
                    rowY{1} = '0';
                    rowX{1} = '0';
                end
                % First extend the large regressor matrix
                largeY = [largeY, sym(zeros(size(largeY,1),numel(rowY)));
                    sym(zeros(1,size(largeY,2))), sym(rowY(:).')];
                largeX = [largeX; (rowX(:))];
                
            end
            
            % remove zero parameters
            zIdx = strcmp(largeX,'0');
            zIdx = find(zIdx);
            largeX(zIdx) = [];
            largeY(:,zIdx) = [];
            
            % Check
            test = simplify(largeY*largeX-inExpr*qd(:));
            if any(test ~= zeros(size(largeY,1),1))
                warning('Warning DynamicsIdentifier.splityx_coriolis has made a mistake!')
            end
        end
        
        function [largeY,largeX] = splityx_gravload(inExpr)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            nRows = size(inExpr,1);
            
            largeX = [];
            largeY = [];
            
            for iRow = 1:nRows
                curEntry = (expand(inExpr(iRow)));
                %     pretty(curEntry);
                
                curString = char(curEntry);
                
                % Split expressions by signs
                [locSigns,allSummands] = regexp(curString,'-|+','match','split');
                if numel(locSigns) < numel(allSummands)
                    locSigns = ['+',locSigns];
                end
                
                
                nSummands = numel(allSummands);
                
                
                if curEntry == sym(0) && nSummands == 1
                    rowY = cell(nSummands);
                    rowY(1) = [];
                    rowX = rowY;
                else
                    rowY = cell(nSummands,1);
                    rowX = cell(nSummands,1);
                    for iSummand = 1:nSummands
                        curSummand = allSummands(iSummand);
                        
                        % Regular expression: match sine or cosine followed by any character
                        % [except a closing bracket] written in brackets followed by possibly
                        % one or none multiplication character \*. This pattern may repeat zero
                        % or more times. The exception for the bracket within the brackets is
                        % necessary to prevent double brackets to be matched.
                        regExp1 = '(sin|cos)'; % sine or cosine followed by
                        regExp2 = '\(';        % a left round bracket followed by
                        regExp3 = '[^\)]*';    % any number of characters except a right round bracket (this is the argument to the sine/cosine), followed by
                        regExp4 = '(\)){1}';   % exactly one right bracket (closes the argument list), followed by
                        regExp5 = '(\^\d+){0,1}';   % possibly a power to the trigonometric function with multiple, followed by
                        regExp6 = '\*{0,1}';       % possibly one power multiplication character, followed by
                        
                        regExpStr = ['(',regExp1,regExp2,regExp3,regExp4,regExp5,regExp6,')*'];
                        
                        %                         regExpStr =
                        %                         '((sin|cos)\([^\)]*(\)){1}(\^\d+){0,1}\*{0,1})*';
                        %                         % original from gravload iros code
                        
                        % extract regressor entry
                        [~, matchstring] = regexp(curSummand,regExpStr,'split','match');
                        if isempty(matchstring{1})
                            matchstring{1} = '1';
                        end
                        rowY(iSummand) = strcat(locSigns(iSummand),'(', matchstring{1},')');
                        
                        % extract parameter
                        param = regexprep(curSummand, regExpStr, '1');
                        rowX{iSummand} = param{1};
                        
                    end
                end
                % Insert found regressors and parameters in large parameter vector and
                % large regressor matrix.
                if isempty(rowY)
                    rowY{1} = '0';
                    rowX{1} = '0';
                end
                
                largeY = [largeY, sym(zeros(size(largeY,1),nSummands));
                    sym(zeros(1,size(largeY,2))), sym(rowY(:).')];
                largeX = [largeX; rowX(:)];
                
            end
            % remove zero parameters
            zIdx = strcmp(largeX,'0');
            zIdx = find(zIdx);
            largeX(zIdx) = [];
            largeY(:,zIdx) = [];
            
            % Check
            test = simplify(largeY*largeX-inExpr);
            if any(test ~= zeros(size(largeY,1),1))
                warning('Warning DynamicsIdentifier.splityx_gravload has made a mistake!')
            end
        end
        
        
        
        function [Y, X] = minimalyx(largeY,largeX)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            [redY,redX] = DynamicsIdentifier.reduceyx(largeY,largeX);
            [Y, X] = DynamicsIdentifier.uniqueyx(redY,redX);
        end
        
        function [redY,redX] = reduceyx(largeY,largeX)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            nRows = size(largeY,1);
            
            % find unique parameters
            allSymPar = sym(largeX);
            if (size(largeY,2)~=numel(largeX))
                allSymPar(allSymPar==0) = [];
            end
            
            [redX,~,idxToRedY] = unique(allSymPar);
            nPar = numel(redX);
            
            % The vector rowredSymPar only contains the unique parameters. The index vector idxTuUnR
            % contains the index of the unique parameter vector uiqueSymPar to whithc
            % the regressor component from rowR belongs. So iterate over all summands
            % and add the summands to their corresponding locations in the unique
            % regressor.
            redY = sym(zeros(nRows,nPar));
            for iRow = 1: nRows
                for iX = 1:numel(allSymPar);
                    redY(iRow,idxToRedY(iX)) = redY(iRow,idxToRedY(iX)) + largeY(iRow,iX);
                end
            end
            
            reduceTest = simplify(redY*redX-largeY*largeX);
            if any(reduceTest ~= zeros(size(redY,1),1))
                warning('Warning DynamicsIdentifier.reduceyx has made a mistake!')
            end
            
        end
        
        function [uniqueY, uniqueX] = uniqueyx(redY,redX)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            uniqueY = sym([]);
            uniqueX = sym([]);
            
            nY = size(redY,2);
            for iY = 1:nY
                isThere = 0;
                nUR = size(uniqueY,2);
                for iUR = 1:nUR
                    tmpMat = [redY(:,iY), uniqueY(:,iUR)];
                    tmpRank = rank(tmpMat);
                    
                    if tmpRank < 2 %nJoints
                        if isempty(findsym((redY(:,iY) ./ uniqueY(:,iUR))))
                            % column is already there: sum parameters
                            if (any((redY(:,iY) ./ uniqueY(:,iUR))<0)) % is positive or negative multiple?
                                uniqueX(iUR,1) = uniqueX(iUR,1) - redX(iY); % negative multiple
                                %                 disp('negative')
                            else
                                uniqueX(iUR,1) = uniqueX(iUR,1) + redX(iY); % positive multiple
                                %                 disp('positive')
                            end
                            isThere = 1;
                        end
                    else
                    end
                end
                
                if isThere == 0
                    % column is new: append
                    uniqueY(:,nUR+1) = redY(:,iY);
                    uniqueX(nUR+1,1) = redX(iY);
                end
            end
            
            uniqueTest = simplify(redY*redX-uniqueY*uniqueX);
            if any(uniqueTest ~= zeros(size(redY,1),1))
                warning('Warning DynamicsIdentifier.uniqueyx has made a mistake!')
            end
        end
        
        % extract inverse dynamics components
        function [I] = extractinertia(inExpr)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            % inExpr must not contain gravity components!
            
            nJoints = size(inExpr,1);
            
            qddString =  sprintf('qdd%d ', 1:nJoints);
            eval(['syms ',  qddString]);
            eval(['symQDD = [', qddString,'].'';']);
            
            qdString =  sprintf('qd%d ', 1:nJoints);
            eval(['syms ',  qdString]);
            eval(['symQD = [', qdString,'].'';']);
            
            I = sym(zeros(nJoints));
            QD = zeros(size(symQD));
            inExpr = subs(inExpr,symQD,QD);
            
            for iCol = 1:nJoints
                
                QDD = sym(zeros(nJoints,1));
                QDD(iCol) = sym(1);
                
                for iRow = 1:nJoints
                    I(iRow,iCol) = subs(inExpr(iRow),symQDD,QDD);
                end
            end
            
            % Check
            inertiaTest = simplify(inExpr-I*symQDD(:));
            if any(inertiaTest ~= zeros(size(nJoints)))
                warning('Warning DynamicsIdentifier.extractinertia has made a mistake!')
            end
            
        end
        
        function [ C ] = extractcoriolis( inExpr )
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
            
            % inExpr must not contain gravity components!
            nJoints = size(inExpr,1);
            
            
            qdString =  sprintf('qd%d ', 1:nJoints);
            eval(['syms ',  qdString]);
            eval(['symQD = [', qdString,'].'';']);
            
            qddString =  sprintf('qdd%d ', 1:nJoints);
            eval(['syms ',  qddString]);
            eval(['symQDD = [', qddString,'].'';']);
            
            QDD = zeros(size(symQDD));
            inExpr = subs(inExpr,symQDD,QDD);
            
            C = sym(zeros(nJoints));
            Csq = sym(zeros(nJoints));
            
            % find the torques that depend on a single finite joint speed,
            % these are due to the squared (centripetal) terms
            %
            %  set QD = [1 0 0 ...] then resulting torque is due to qd_1^2
            for j=1:nJoints
                QD = sym(zeros(1,nJoints));
                QD(j) = 1;
                %     tau = robot2.rne(q, QD, zeros(size(q)), [0 0 0]');
                tauLocal = sym(zeros(nJoints,1));
                for iRow = 1:nJoints
                    %     tauLocal = subs(tau,[qd1,qd2,qd3],QD);
                    tauLocal(iRow) = subs(inExpr(iRow),symQD.',QD);
                end
                Csq(:,j) = Csq(:,j) + tauLocal;
            end
            
            % find the torques that depend on a pair of finite joint speeds,
            % these are due to the product (Coridolis) terms
            %  set QD = [1 1 0 ...] then resulting torque is due to
            %    qd_1 qd_2 + qd_1^2 + qd_2^2
            for j=1:nJoints
                for k=j+1:nJoints
                    % find a product term  qd_j * qd_k
                    QD = sym(zeros(1,nJoints));
                    QD(j) = 1;
                    QD(k) = 1;
                    
                    for iRow = 1:nJoints
                        tauLocal(iRow) = subs(inExpr(iRow),symQD.',QD);
                    end
                    C(:,k) = C(:,k) + (tauLocal - Csq(:,k) - Csq(:,j)) * symQD(j);
                end
            end
            C = C + Csq * diag(symQD);
            
        end
        
        % ----
        
        function [y] = reshape_id_input(x)
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
                   
            nRows = size(x,1);
            nJoints = size(x,2);
            
            y = zeros(nRows*nJoints,1);
            
            for iJoint = 1:nJoints
                y(iJoint:nJoints:end,1) = x(:,iJoint);
            end
            
        end
        
        % ----
        
        function [Ynum] = numericregressor(Y,varargin)
            % DYNAMICSIDENTIFIER.NUMERICREGRESSOR Concatenates regressors for
            % a given set of identification data samples.
            %
            %   [Ynum, Tnum] = numericregressor(Y,T,Q,QD,QDD])
            %
            %
            % Author::
            %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
            %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
            %  www.iit.it
            %
            % See also DynamicsIdentifier
        
            nJoints = size(Y,1);
            nPar = size(Y,2);
            
            Q = [];
            Qd = [];
            Qdd = [];
            
            if nargin < 2
                error('You should provide some data, if you want me to generate a numerical regressor...')
            else
            end
            
            if nargin > 1
                Q = varargin{1};
                if size(Q,2) ~= nJoints
                    error('Input dimension mismatch: Q should be of size [nDataPoints x nJoints]');
                end
                for iJoint = 1:nJoints
                    eval(sprintf('q%d = Q(:,%d);',iJoint,iJoint));
                end
            end
            if nargin > 2
                Qd = varargin{2};
                if size(Qd,2) ~= nJoints
                    error('Input dimension mismatch: Qd should be of size [nDataPoints x nJoints]');
                end
                for iJoint = 1:nJoints
                    
                    eval(sprintf('qd%d = Qd(:,%d);',iJoint,iJoint));
                    
                end
            end
            if nargin > 3
                Qdd = varargin{3};
                if size(Qdd,2) ~= nJoints
                    error('Input dimension mismatch: Qdd should be of size [nDataPoints x nJoints]');
                end
                for iJoint = 1:nJoints
                    
                    eval(sprintf('qdd%d = Qdd(:,%d);',iJoint,iJoint));
                end
            end
            
            %initialize variables
            nVals = size(Q,1);
            Ynum = zeros(nVals*nJoints,nPar);
            
            for iJoint = 1:nJoints
                curStr = char((Y(iJoint,:)));
                curStr = strrep(curStr,'*','.*');
                curStr = strrep(curStr,'^','.^');
                curStr = strrep(curStr,' 0,',' zeros(nVals,1),');
                curStr = strrep(curStr,'[0,',' [zeros(nVals,1),');
                curStr = strrep(curStr,' 0]',' zeros(nVals,1)]');
                curStr = strrep(curStr,' 1]',' ones(nVals,1)]');
                curStr = strrep(curStr,'matrix','');
                
                Ynum(iJoint:nJoints:end,:) = eval(curStr);
            end
            
            
        end
        
        % ----
        
        function [Ynum, Tnum] = getregressionproblem(Y,T,varargin)
        %
        % Author::
        %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
        %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
        %  www.iit.it
        %
        % See also DynamicsIdentifier
        
            
            nJoints = size(Y,1);
            
            if nargin < 2
                error('You should provide some data, if you want me to generate a numerical regressor...')
            else
            end
            if nargin > 1
                T = varargin{1};
                if size(T,2) ~= nJoints
                    error('Input dimension mismatch: T should be of size [nDataPoints x nJoints]');
                end
            end
            
            if nargin > 2
                Q = varargin{1};
                if size(Q,2) ~= nJoints
                    error('Input dimension mismatch: Q should be of size [nDataPoints x nJoints]');
                end
                for iJoint = 1:nJoints
                    eval(sprintf('q%d = Q(:,%d);',iJoint,iJoint));
                end
            end
            if nargin > 3
                Qd = varargin{2};
                if size(Qd,2) ~= nJoints
                    error('Input dimension mismatch: Qd should be of size [nDataPoints x nJoints]');
                end
                for iJoint = 1:nJoints
                    
                    eval(sprintf('qd%d = Qd(:,%d);',iJoint,iJoint));
                    
                end
            end
            if nargin > 4
                Qdd = varargin{3};
                if size(Qdd,2) ~= nJoints
                    error('Input dimension mismatch: Qdd should be of size [nDataPoints x nJoints]');
                end
                for iJoint = 1:nJoints
                    
                    eval(sprintf('qdd%d = Qdd(:,%d);',iJoint,iJoint));
                end
            end
            
            
            Tnum = DynamicsIdentifier.reshape_id_input(T);
            Ynum = DynamicsIdentifier.numericregressor(Y,varargin{:});
            
        end
        
        % ----
        
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Regressor scaling and optimization (requires Optimization Toolbox)
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [YNum, fval]= scaledrandregressor(Y,nSamp,varargin)
        %
        % Author::
        %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
        %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
        %  www.iit.it
        %
        % See also DynamicsIdentifier
        
            doOptimizeRegressor = 0;
            if nargin > 2
                doOptimizeRegressor = varargin{1};
            end
            
            nJoints = size(Y,1);
            
            %% generate some random joint values (positions and derivatives)
            % within these boundaries
            blq = -1*pi; % lower position boundary
            bhq =  1*pi; % upper position boundary
            blqd = -2*pi; % lower velocity boundary
            bhqd =  2*pi; % upper velocity boundary
            blqdd = -4*pi; % lower accerleration boundary
            bhqdd =  4*pi; % upper acceleration boundary
            
            % position
            rng('shuffle'); % create new seed for random number generator
            q =  blq + (bhq-blq).*rand(nSamp,nJoints);
            
            % velocity
            rng('shuffle'); % create new seed for random number generator
            qd =  blqd + (bhqd-blqd).*rand(nSamp,nJoints);

            % acceleration
            rng('shuffle'); % create new seed for random number generator
            qdd =  blqdd + (bhqdd-blqdd).*rand(nSamp,nJoints);
            
            % collect all joint values in an optimization matrix
            x = [q(:);qd(:);qdd(:)];
            
            if exist('fminunc.m','file') && doOptimizeRegressor
                
                x0 = x;
                
                % create components of the constraint inequality A x < b
                Ai = [eye(3*nJoints*nSamp); -eye(3*nJoints*nSamp)];
                A = repmat(Ai,3,1);
                
                bi2 = ones(2*3*nJoints*nSamp,1);
                b = [bhq*bi2; bhqd*bi2; bhqdd*bi2];
                
                % optimize the regressor under given constraints!
                fun = @(x)DynamicsIdentifier.rand_regressor_cost_function(x,Y);
                [x, fval] = fmincon(fun, x0, A, b);
            else
                fval = NaN;
               % just use the unoptimized random numerical regressor 
            end
            % assemble the best found regressor
            YNum = DynamicsIdentifier.numericregressor(Y,... regressor
                reshape(x(1:nJoints*nSamp),nSamp,nJoints),...                 % q
                reshape(x((1:nJoints*nSamp)+nJoints*nSamp),nSamp,nJoints),... % qd
                reshape(x((1:nJoints*nSamp)+2*nJoints*nSamp),nSamp,nJoints)); % qdd
            
        end
        
        % ----
        
        function J = rand_regressor_cost_function(x,Y)
        %
        % Author::
        %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
        %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
        %  www.iit.it
        %
        % See also DynamicsIdentifier
        
            nJoints = size(Y,1);
            nSamp = numel(x)/nJoints/3;
            
            % assemble regressor
            YNum = DynamicsIdentifier.numericregressor(Y,... regressor
                reshape(x(1:nJoints*nSamp),nSamp,nJoints),...                 % q
                reshape(x((1:nJoints*nSamp)+nJoints*nSamp),nSamp,nJoints),... % qd
                reshape(x((1:nJoints*nSamp)+2*nJoints*nSamp),nSamp,nJoints)); % qdd
            
            % Criteria as used in 
            % M. Gautier: Numerical Calculation of the Base Inertial Parameters
            % of Robots. Journal of Robotic Systems, Vol. 8(4), August 1991, p.
            % 485-506.
            % Weighting factors are ommitted since there is no general rule
            % to tune them... Equal weighting of both criteria does the
            % job.
            S = max(max(abs(YNum))) / min(min(abs(YNum(YNum~=0))));
            J = norm(YNum,'fro')  + S;
        end
        
        % ----
        
        function [Y,X, fval] = std2baseyx(stdY, stdX, varargin)
        % DynamicsIdentifier.std2baseyx Compute base regressor and 
        % paramter vector from their standard counterparts.
        % 
        % The function derives the vector of dynamic base parameters
        % together with the corresponding regression matrix given a minimal
        % regressor and parameter vector expressed in standard inertial
        % parameters. 
        % The technique to achieve that is taken from:
        %
        % M. Gautier: Numerical Calculation of the Base Inertial Parameters
        % of Robots. Journal of Robotic Systems, Vol. 8(4), August 1991, p.
        % 485-506.
        %
        % It is based on the analysis of the span of a numerical version of
        % the standard regressor. The numerical version is obtained using
        % random joint values. If the optimization toolbox is installed,
        % the function can optimizes the values of this numerical
        % regressor, in order to obtain improve accuracy of the results. 
        % Since the optimization problem is high dimensional, this solution
        % can be time consuming. Therefore, it is switched off by default.
        % It can be activated by passing a flag to the function.
        %
        % [YB,XB, fval] = getbaseyx(stdY, stdX {,flag})
        %
        % stdY: symbolic regression matrix expressed with respect to the
        %       standard inertial parameters
        % stdX: vector of standard inertial parameters     
        % flag: 1 --> regressor optimization on, 0 --> off (default)
        %
        % YB:   symbolic regression matrix expressed with respect to the
        %       base inertial parameters
        % XB:   vector of base inertial parameters
        % fval: fitness value of the random numeric regressor used for
        %       finding the set of base parameters
        %
        % Author::
        %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
        %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
        %  www.iit.it
        %
        % See also DynamicsIdentifier
            
            % Get a properly scaled random numeric regressor
            nc = numel(stdX);
            nSamp = nc;
            
            doOptimizeRegressor = 0;
            if nargin > 2
                doOptimizeRegressor = varargin{1};
            end
            
            [Ynum, fval] = DynamicsIdentifier.scaledrandregressor(stdY,nSamp,doOptimizeRegressor);
            
            %% perform a numerical span analysis based on QR decomposition
            [~,Rp,P] = qr(Ynum);
            
            % Rp is a upper right triangular matrix. The diagonal
            % elements are in non ascending order.
            % Find those diagonal elements which are below the relative
            % machine precision. The threshold for that test computes to:
            thres = nSamp * eps * abs(Rp(1,1));
            idxDel = find( abs(diag(Rp)) < thres );
            
            % The number of base parameters is now nb, the number of 
            % diagonal elements that are above the relative machine 
            % precision:
            nb = nc-numel(idxDel);
            
            % Partition R into full rank R1 and R2
            R1 = Rp(1:nb,1:nb); % the columns of R1 belong to independent 
                                % parameters
            R2 = Rp(1:nb,nb+1:end); % the columns of R2 belong to the 
                                    % dependent paramters
            
            % Compute combination matrix for the dependent parameters
            invR1R2 = R1\R2; % R1 is full rank and square. Inverse exists.
            invR1R2(abs(invR1R2)<1e-13) = 0; % Hardcoded threshold should 
                                             % be class property in the 
                                             % future
            
            %% Now rearrange the symbolic input according to our findings
            % Apply permutation matrix P
            X12 = P.'*stdX;
            Y12 = stdY*P;
            
            X = X12(1:nb) + invR1R2*X12(nb+1:end);
            Y = Y12(:,1:nb);
            
            %% Check if the derived base set is correct:
            testA = Y12*X12;
            testB = Y*X;
            if any( simplify(testA-testB) ) 
                warning('The derived set of base parameters might have errors!')
            end
            
        end
        
        % ----
        
        function rob = preparerobot(rob)
        % DynamicsIdentifier.preparerobot Fills inertial parameters with
        % symbolic variables.
        %
        %  prob = preparerobot(rob)
        %
        %
        % Author::
        %  Joern Malzahn (joern.malzahn@tu-dortmund.de)
        %  2015 IIT Istituto Italiano di Tecnologia, Genova, Italy.
        %  www.iit.it
        %
        % See also CodeGenerator, DynamicsIdentifier.
            nJoints = rob.n;

            for iJoints = 1:nJoints
                
                % create symbolic variables
                cmdString = sprintf('syms rx%d ry%d rz%d m%d Jxx%d Jyy%d Jzz%d Jxy%d Jxz%d Jyz%d', iJoints*ones(1,10));
                eval(cmdString);
                
                cmdString = sprintf('[rx%d; ry%d; rz%d];', iJoints*ones(1,3));
                R = eval(cmdString);
                
                cmdString = sprintf('[Jxx%d, Jxy%d, Jxz%d; Jxy%d, Jyy%d, Jyz%d; Jxz%d, Jyz%d, Jzz%d];', iJoints*ones(1,9));
                J = eval(cmdString);
                
                cmdString = sprintf('m%d', iJoints);
                m = eval(cmdString);
                
                % Apply the parallel axes theorem to make the dynamics
                % identifier work with joint referred inertia parameters J
                I = J - m * (R.'*R*eye(3) - R*R.');
                
                rob.links(iJoints).I = I;
                rob.links(iJoints).r = R;
                rob.links(iJoints).m = m;
                
            end
        end
            
        % ----

    end
    
end

