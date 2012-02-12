function [handles,levels,parentIdx,listing] = findjobj(container,varargin)
%findjobj Find java objects contained within a specified java container or Matlab GUI handle
%
% Syntax:
%    [handles, levels, parentIds, listing] = findjobj(container, 'PropName',PropValue(s), ...)
%
% Input parameters:
%    container - optional handle to java container uipanel or figure. If unsupplied then current figure will be used
%    'PropName',PropValue - optional list of property pairs (case insensitive). PropName may also be named -PropName
%         'position' - filter results based on those elements that contain the specified X,Y position or a java element
%                      Note: specify a Matlab position (X,Y = pixels from bottom left corner), not a java one
%         'size'     - filter results based on those elements that have the specified W,H (in pixels)
%         'class'    - filter results based on those elements that contain the substring  (or java class) PropValue
%                      Note1: filtering is case insensitive and relies on regexp, so you can pass wildcards etc.
%                      Note2: '-class' is an undocumented findobj PropName, but only works on Matlab (not java) classes
%         'property' - filter results based on those elements that possess the specified case-insensitive property string
%                      Note1: passing a property value is possible if the argument following 'property' is a cell in the
%                             format of {'propName','propValue'}. Example: FINDJOBJ(...,'property',{'Text','click me'})
%                      Note2: partial property names (e.g. 'Tex') are accepted, as long as they're not ambiguous
%         'depth'    - filter results based on specified depth. 0=top-level, Inf=all levels (default=Inf)
%         'flat'     - same as specifying: 'depth',0
%         'not'      - negates the following filter: 'not','class','c' returns all elements EXCEPT those with class 'c'
%         'persist'  - persist figure components information, allowing much faster results for subsequent invocations
%         'print'    - display all java elements in a hierarchical list, indented appropriately
%                      Note1: optional PropValue of element index or handle to java container
%                      Note2: normally this option would be placed last, after all filtering is complete. Placing this
%                             option before some filters enables debug print-outs of interim filtering results.
%                      Note3: output is to the Matlab command window unless the 'listing' (4th) output arg is requested
%         'list'     - same as 'print'
%         'nomenu'   - skip menu processing, for "lean" list of handles & much faster processing;
%                      This option is the default for HG containers but not for figure, Java or no container
%         'debug'    - list found component positions in the Command Window
%
% Output parameters:
%    handles   - list of handles to java elements
%    levels    - list of corresponding hierarchy level of the java elements (top=0)
%    parentIds - list of indexes (in unfiltered handles) of the parent container of the corresponding java element
%    listing   - results of 'print'/'list' options (empty if these options were not specified)
%
%    Note: If no output parameter is specified, then an interactive window will be displayed with a
%    ^^^^  tree view of all container components, their properties and callbacks.
%
% Examples:
%    findjobj;                     % display list of all javaelements of currrent figure in an interactive GUI
%    handles = findjobj;           % get list of all java elements of current figure (inc. menus, toolbars etc.)
%    findjobj('print');            % list all java elements in current figure
%    findjobj('print',6);          % list all java elements in current figure, contained within its 6th element
%    handles = findjobj(hButton);                                     % hButton is a matlab button
%    handles = findjobj(gcf,'position',getpixelposition(hButton,1));  % same as above but also return hButton's panel
%    handles = findjobj(hButton,'persist');                           % same as above, persist info for future reuse
%    handles = findjobj('class','pushbutton');                        % get all pushbuttons in current figure
%    handles = findjobj('class','pushbutton','position',123,456);     % get all pushbuttons at the specified position
%    handles = findjobj(gcf,'class','pushbutton','size',23,15);       % get all pushbuttons with the specified size
%    handles = findjobj('property','Text','not','class','button');    % get all non-button elements with 'text' property
%    handles = findjobj('-property',{'Text','click me'});             % get all elements with 'text' property = 'click me'
%
% Sample usage:
%    hButton = uicontrol('string','click me');
%    jButton = findjobj(hButton,'nomenu');
%      % or: jButton = findjobj('property',{'Text','click me'});
%    jButton.setFlyOverAppearance(1);
%    jButton.setCursor(java.awt.Cursor.getPredefinedCursor(java.awt.Cursor.HAND_CURSOR));
%    set(jButton,'FocusGainedCallback',@myMatlabFunction);   % some 30 callback points available...
%    jButton.get;   % list all changeable properties...
%
%    hEditbox = uicontrol('style',edit');
%    jEditbox = findjobj(hEditbox,'nomenu');
%    jEditbox.setCaretColor(java.awt.Color.red);
%    jEditbox.KeyTypedCallback = @myCallbackFunc;  % many more callbacks where this came from...
%    jEdit.requestFocus;
%
% Known issues/limitations:
%    - Cannot currently process multiple container objects - just one at a time
%    - Initial processing is a bit slow when the figure is laden with many UI components (so better use 'persist')
%    - Passing a simple container Matlab handle is currently filtered by its position+size: should find a better way to do this
%    - Matlab uipanels are not implemented as simple java panels, and so they can't be found using this utility
%    - Labels have a write-only text property in java, so they can't be found using the 'property',{'Text','string'} notation
%
% Warning:
%    This code heavily relies on undocumented and unsupported Matlab functionality.
%    It works on Matlab 7+, but use at your own risk!
%
% Bugs and suggestions:
%    Please send to Yair Altman (altmany at gmail dot com)
%
% Change log:
%    2009-05-24: Added support for future Matlab versions that will not support JavaFrame
%    2009-05-15: Added sanity checks for axes items
%    2009-04-28: Added 'debug' input arg; increased size tolerance 1px => 2px
%    2009-04-23: Fixed location of popupmenus (always 20px high despite what's reported by Matlab...); fixed uiinspect processing issues; added blog link; narrower action buttons
%    2009-04-09: Automatic 'nomenu' for uicontrol inputs; significant performance improvement
%    2009-03-31: Fixed position of some Java components; fixed properties tooltip; fixed node visibility indication
%    2009-02-26: Indicated components visibility (& auto-collapse non-visible containers); auto-highlight selected component; fixes in node icons, figure title & tree refresh; improved error handling; display FindJObj version update description if available
%    2009-02-24: Fixed update check; added dedicated labels icon
%    2009-02-18: Fixed compatibility with old Matlab versions
%    2009-02-08: Callbacks table fixes; use uiinspect if available; fix update check according to new FEX website
%    2008-12-17: R2008b compatibility
%    2008-09-10: Fixed minor bug as per Johnny Smith
%    2007-11-14: Fixed edge case problem with class properties tooltip; used existing object icon if available; added checkbox option to hide standard callbacks
%    2007-08-15: Fixed object naming relative property priorities; added sanity check for illegal container arg; enabled desktop (0) container; cleaned up warnings about special class objects
%    2007-08-03: Fixed minor tagging problems with a few Java sub-classes; displayed UIClassID if text/name/tag is unavailable
%    2007-06-15: Fixed problems finding HG components found by J. Wagberg
%    2007-05-22: Added 'nomenu' option for improved performance; fixed 'export handles' bug; fixed handle-finding/display bugs; "cleaner" error handling
%    2007-04-23: HTMLized classname tooltip; returned top-level figure Frame handle for figure container; fixed callbacks table; auto-checked newer version; fixed Matlab 7.2 compatibility issue; added HG objects tree
%    2007-04-19: Fixed edge case of missing figure; displayed tree hierarchy in interactive GUI if no output args; workaround for figure sub-menus invisible unless clicked
%    2007-04-04: Improved performance; returned full listing results in 4th output arg; enabled partial property names & property values; automatically filtered out container panels if children also returned; fixed finding sub-menu items
%    2007-03-20: First version posted on the MathWorks file exchange: <a href="http://www.mathworks.com/matlabcentral/fileexchange/loadFile.do?objectId=14317">http://www.mathworks.com/matlabcentral/fileexchange/loadFile.do?objectId=14317</a>
%
% See also:
%    java, handle, findobj, findall, javaGetHandles, uiinspect (on the File Exchange)

% License to use and modify this code is granted freely to all interested, as long as the original author is
% referenced and attributed as such. The original author maintains the right to be solely associated with this work.

% Programmed and Copyright by Yair M. Altman: altmany(at)gmail.com
% $Revision: 1.21 $  $Date: 2009/05/24 00:47:55 $

    % Ensure Java AWT is enabled
    error(javachk('awt'));

    persistent pContainer pHandles pLevels pParentIdx pPositions

    try
        % Initialize
        handles = handle([]);
        levels = [];
        parentIdx = [];
        positions = [];   % Java positions start at the top-left corner
        %sizes = [];
        listing = '';
        hg_levels = [];
        hg_handles = handle([]);  % HG handles are double
        hg_parentIdx = [];
        nomenu = false;
        menuBarFoundFlag = false;

        % Default container is the current figure's root panel
        if nargin
            if isempty(container)  % empty container - bail out
                return;
            elseif ischar(container)  % container skipped - this is part of the args list...
                varargin = {container, varargin{:}};
                origContainer = getCurrentFigure;
                [container,contentSize] = getRootPanel(origContainer);
            elseif isequal(container,0)  % root
                origContainer = handle(container);
                container = com.mathworks.mde.desk.MLDesktop.getInstance.getMainFrame;
                contentSize = [container.getWidth, container.getHeight];
            elseif ishghandle(container) % && ~isa(container,'java.awt.Container')
                container = container(1);  % another current limitation...
                hFig = ancestor(container,'figure');
                origContainer = handle(container);
                if isa(origContainer,'uimenu')
                    % getpixelposition doesn't work for menus... - damn!
                    varargin = {'class','MenuPeer', 'property',{'Label',strrep(get(container,'Label'),'&','')}, varargin{:}};
                elseif ~isa(origContainer, 'figure') && ~isempty(hFig)
                    % See limitations section above: should find a better way to directly refer to the element's java container
                    try
                        % Note: 'PixelBounds' is undocumented and unsupported, but much faster than getpixelposition!
                        % ^^^^  unfortunately, its Y position is inaccurate in some cases - damn!
                        %size = get(container,'PixelBounds');
                        pos = fix(getpixelposition(container,1));
                        %varargin = {'position',pos(1:2), 'size',pos(3:4), 'not','class','java.awt.Panel', varargin{:}};
                    catch
                        try
                            figName = get(hFig,'name');
                            if strcmpi(get(hFig,'number'),'on')
                                figName = regexprep(['Figure ' num2str(hFig) ': ' figName],': $','');
                            end
                            mde = com.mathworks.mde.desk.MLDesktop.getInstance;
                            jFig = mde.getClient(figName);
                            if isempty(jFig), error('dummy');  end
                        catch
                            warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');  % R2008b compatibility
                            jFig = get(get(hFig,'JavaFrame'),'FigurePanelContainer');
                        end
                        pos = [];
                        try
                            pxsize = get(container,'PixelBounds');
                            pos = [pxsize(1)+5, jFig.getHeight - (pxsize(4)-5)];
                        catch
                            % never mind...
                        end
                    end
                    if size(pos,2) == 2
                        pos(:,3:4) = 0;
                    end
                    if ~isempty(pos)
                        if isa(handle(container),'uicontrol') && strcmp(get(container,'style'),'popupmenu')
                            % popupmenus (combo-box dropdowns) are always 20px high
                            pos(2) = pos(2) + pos(4) - 20;
                            pos(4) = 20;
                        end
                        %varargin = {'position',pos(1:2), 'size',size(3:4)-size(1:2)-10, 'not','class','java.awt.Panel', varargin{:}};
                        varargin = {'position',pos(1:2)+[0,pos(4)], 'size',pos(3:4), 'not','class','java.awt.Panel', 'nomenu', varargin{:}};
                    end
                elseif isempty(hFig)
                    hFig = handle(container);
                end
                [container,contentSize] = getRootPanel(hFig);
            elseif isjava(container)
                % Maybe a java container obj (will crash otherwise)
                origContainer = container;
                contentSize = [container.getWidth, container.getHeight];
            else
                error('YMA:findjobj:IllegalContainer','Input arg does not appear to be a valid GUI object');
            end
        else
            % Default container = current figure
            origContainer = getCurrentFigure;
            [container,contentSize] = getRootPanel(origContainer);
        end

        % Check persistency
        if isequal(pContainer,container)
            % persistency requested and the same container is reused, so reuse the hierarchy information
            [handles,levels,parentIdx,positions] = deal(pHandles, pLevels, pParentIdx, pPositions);
        else
            % Pre-allocate space of complex data containers for improved performance
            handles = repmat(handles,1,1000);
            positions = zeros(1000,2);

            % Check whether to skip menu processing
            nomenu = paramSupplied(varargin,'nomenu');

            % Traverse the container hierarchy and extract the elements within
            traverseContainer(container,0,1);

            % Remove unnecessary pre-allocated elements
            dataLen = length(levels);
            handles  (dataLen+1:end) = [];
            positions(dataLen+1:end,:) = [];
        end

        % Process persistency check before any filtering is done
        if paramSupplied(varargin,'persist')
            [pContainer, pHandles, pLevels, pParentIdx, pPositions] = deal(container,handles,levels,parentIdx,positions);
        end

        % Save data for possible future use in presentObjectTree() below
        allHandles = handles;
        allLevels  = levels;
        allParents = parentIdx;
        selectedIdx = 1:length(handles);
        %[positions(:,1)-container.getX, container.getHeight - positions(:,2)]

        % Debug-list all found compponents and their positions
        if paramSupplied(varargin,'debug')
            for handleIdx = 1 : length(allHandles)
                pos = sprintf('%d,%d %dx%d',[positions(handleIdx,:) getXY(handles(handleIdx))]);
                disp([repmat(' ',1,levels(handleIdx)) '[' pos(1:end-1) '] ' char(toString(handles(handleIdx)))]);
            end
        end

        % Process optional args
        % Note: positions is NOT returned since it's based on java coord system (origin = top-left): will confuse Matlab users
        processArgs(varargin{:});  %#ok

        % De-cell and trim listing, if only one element was found (no use for indented listing in this case)
        if iscell(listing) && length(listing)==1
            listing = strtrim(listing{1});
        end

        % If no output args and no listing requested, present the FINDJOBJ interactive GUI
        if nargout == 0 && isempty(listing)
            presentObjectTree();

            % Display the listing, if this was specifically requested yet no relevant output arg was specified
        elseif nargout < 4 && ~isempty(listing)
            if ~iscell(listing)
                disp(listing);
            else
                for listingIdx = 1 : length(listing)
                    disp(listing{listingIdx});
                end
            end
        end

        return;  %debug point

    catch
        % 'Cleaner' error handling - strip the stack info etc.
        err = lasterror;
        err.message = regexprep(err.message,'Error using ==> [^\n]+\n','');
        if isempty(findstr(mfilename,err.message))
            % Indicate error origin, if not already stated within the error message
            err.message = [mfilename ': ' err.message];
        end
        rethrow(err);
    end


%% Check existence of a (case-insensitive) optional parameter in the params list
    function [flag,idx] = paramSupplied(paramsList,paramName)
        idx = find(~cellfun('isempty',regexpi(paramsList(cellfun(@ischar,paramsList)),['^-?' paramName])));
        flag = any(idx);
    end

%% Get current figure (even if its 'HandleVisibility' property is 'off')
    function curFig = getCurrentFigure
        oldShowHidden = get(0,'ShowHiddenHandles');
        set(0,'ShowHiddenHandles','on');  % minor fix per Johnny Smith
        curFig = gcf;
        set(0,'ShowHiddenHandles',oldShowHidden);
    end

%% Get Java reference to top-level (root) panel - actually, a reference to the java figure
    function [jRootPane,contentSize] = getRootPanel(hFig)
        try
            contentSize = [0,0];  % initialize
            jRootPane = hFig;
            figName = get(hFig,'name');
            if strcmpi(get(hFig,'number'),'on')
                figName = regexprep(['Figure ' num2str(hFig) ': ' figName],': $','');
            end
            mde = com.mathworks.mde.desk.MLDesktop.getInstance;
            jFigPanel = mde.getClient(figName);
            jRootPane = jFigPanel;
            jRootPane = jFigPanel.getRootPane;
        catch
            try
                jFrame = get(hFig,'JavaFrame');
                jFigPanel = get(jFrame,'FigurePanelContainer');
                jRootPane = jFigPanel;
                jRootPane = jFigPanel.getComponent(0).getRootPane;
            catch
                % Never mind
            end
        end
        try
            % If invalid RootPane, retry up to N times
            tries = 10;
            while isempty(jRootPane) && tries>0  % might happen if figure is still undergoing rendering...
                drawnow; pause(0.001);
                tries = tries - 1;
                jRootPane = jFigPanel.getComponent(0).getRootPane;
            end

            % If still invalid, use FigurePanelContainer which is good enough in 99% of cases... (menu/tool bars won't be accessible, though)
            if isempty(jRootPane)
                jRootPane = jFigPanel;
            end
            contentSize = [jRootPane.getWidth, jRootPane.getHeight];

            % Try to get the ancestor FigureFrame
            jRootPane = jRootPane.getTopLevelAncestor;
        catch
            % Never mind - FigurePanelContainer is good enough in 99% of cases... (menu/tool bars won't be accessible, though)
        end
    end

%% Traverse the container hierarchy and extract the elements within
    function traverseContainer(jcontainer,level,parent)
        persistent figureComponentFound menuRootFound

        % Record the data for this node
        %disp([repmat(' ',1,level) '<= ' char(jcontainer.toString)])
        thisIdx = length(levels) + 1;
        levels(thisIdx) = level;
        parentIdx(thisIdx) = parent;
        handles(thisIdx) = handle(jcontainer,'callbackproperties');
        try
            positions(thisIdx,:) = getXY(jcontainer);
            %sizes(thisIdx,:) = [jcontainer.getWidth, jcontainer.getHeight];
        catch
            positions(thisIdx,:) = [0,0];
            %sizes(thisIdx,:) = [0,0];
        end
        if level>0
            positions(thisIdx,:) = positions(thisIdx,:) + positions(parent,:);
            if ~figureComponentFound && ...
               strcmp(jcontainer.getName,'fComponentContainer') && ...
               isa(jcontainer,'com.mathworks.hg.peer.FigureComponentContainer')   % there are 2 FigureComponentContainers - only process one...

                % restart coordinate system, to exclude menu & toolbar areas
                positions(thisIdx,:) = positions(thisIdx,:) - [jcontainer.getRootPane.getX, jcontainer.getRootPane.getY];
                figureComponentFound = true;
            end
        elseif level==1
            positions(thisIdx,:) = positions(thisIdx,:) + positions(parent,:);
        else
            % level 0 - initialize flags used later
            figureComponentFound = false;
            menuRootFound = false;
        end
        parentId = length(parentIdx);

        % Traverse Menu items, unless the 'nomenu' option was requested
        if ~nomenu
            try
                for child = 1 : getNumMenuComponents(jcontainer)
                    traverseContainer(jcontainer.getMenuComponent(child-1),level+1,parentId);
                end
            catch
                % Probably not a Menu container, but maybe a top-level JMenu, so discard duplicates
                %if isa(handles(end).java,'javax.swing.JMenuBar')
                if ~menuRootFound && strcmp(handles(end).java.class,'javax.swing.JMenuBar')  %faster...
                    if removeDuplicateNode(thisIdx)
                        menuRootFound = true;
                        return;
                    end
                end
            end
        end

        % Now recursively process all this node's children (if any), except menu items if so requested
        %if isa(jcontainer,'java.awt.Container')
        try  % try-catch is faster than checking isa(jcontainer,'java.awt.Container')...
            %if jcontainer.getComponentCount,  jcontainer.getComponents,  end
            if ~nomenu || menuBarFoundFlag || isempty(strfind(jcontainer.class,'FigureMenuBar'))
                lastChildComponent = java.lang.Object;
                child = 0;
                while (child < jcontainer.getComponentCount)
                    childComponent = jcontainer.getComponent(child);
                    % Looping over menus sometimes causes jcontainer to get mixed up (probably a JITC bug), so identify & fix
                    if isequal(childComponent,lastChildComponent)
                        child = child + 1;
                        childComponent = jcontainer.getComponent(child);
                    end
                    lastChildComponent = childComponent;
                    %disp([repmat(' ',1,level) '=> ' num2str(child) ': ' char(childComponent.class)])
                    traverseContainer(childComponent,level+1,parentId);
                    child = child + 1;
                end
            else
                menuBarFoundFlag = true;  % use this flag to skip further testing for FigureMenuBar
            end
        catch
            % do nothing - probably not a container
            %dispError
        end

        % ...and yet another type of child traversal...
        try
            for child = 1 : jcontainer.java.getChildCount
                traverseContainer(jcontainer.java.getChildAt(child-1),level+1,parentId);
            end
        catch
            % do nothing - probably not a container
            %dispError
        end

        % TODO: Add axis (plot) component handles
    end

%% Get the XY location of a Java component
    function xy = getXY(jcontainer)
            % Note: getX/getY are better than get(..,'X') (mem leaks),
            % ^^^^  but sometimes they fail and revert to getx.m ...
            % Note2: try awtinvoke() catch is faster than checking ismethod()...
            % Note3: using AWTUtilities.invokeAndWait() directly is even faster than awtinvoke()...
            try %if ismethod(jcontainer,'getX')
                %positions(thisIdx,:) = [jcontainer.getX, jcontainer.getY];
                cls = getClass(jcontainer);
                location = com.mathworks.jmi.AWTUtilities.invokeAndWait(jcontainer,getMethod(cls,'getLocation',[]),[]);
                x = location.getX;
                y = location.getY;
            catch %else
                try
                    x = com.mathworks.jmi.AWTUtilities.invokeAndWait(jcontainer,getMethod(cls,'getX',[]),[]);
                    y = com.mathworks.jmi.AWTUtilities.invokeAndWait(jcontainer,getMethod(cls,'getY',[]),[]);
                catch
                    try
                        x = awtinvoke(jcontainer,'getX()');
                        y = awtinvoke(jcontainer,'getY()');
                    catch
                        x = get(jcontainer,'X');
                        y = get(jcontainer,'Y');
                    end
                end
            end
            %positions(thisIdx,:) = [x, y];
            xy = [x,y];
    end

%% Get the number of menu sub-elements
    function numMenuComponents  = getNumMenuComponents(jcontainer)

        % The following line will raise an Exception for anything except menus
        numMenuComponents = jcontainer.getMenuComponentCount;

        % No error so far, so this must be a menu container...
        % Note: Menu subitems are not visible until the top-level (root) menu gets initial focus...
        % Try several alternatives, until we get a non-empty menu (or not...)
        % TODO: Improve performance - this takes WAY too long...
        if jcontainer.isTopLevelMenu && (numMenuComponents==0)
            jcontainer.requestFocus;
            numMenuComponents = jcontainer.getMenuComponentCount;
            if (numMenuComponents == 0)
                drawnow; pause(0.001);
                numMenuComponents = jcontainer.getMenuComponentCount;
                if (numMenuComponents == 0)
                    jcontainer.setSelected(true);
                    numMenuComponents = jcontainer.getMenuComponentCount;
                    if (numMenuComponents == 0)
                        drawnow; pause(0.001);
                        numMenuComponents = jcontainer.getMenuComponentCount;
                        if (numMenuComponents == 0)
                            jcontainer.doClick;  % needed in order to populate the sub-menu components
                            numMenuComponents = jcontainer.getMenuComponentCount;
                            if (numMenuComponents == 0)
                                drawnow; %pause(0.001);
                                numMenuComponents = jcontainer.getMenuComponentCount;
                                jcontainer.doClick;  % close menu by re-clicking...
                                if (numMenuComponents == 0)
                                    drawnow; %pause(0.001);
                                    numMenuComponents = jcontainer.getMenuComponentCount;
                                end
                            else
                                % ok - found sub-items
                                % Note: no need to close menu since this will be done when focus moves to the FindJObj window
                                %jcontainer.doClick;  % close menu by re-clicking...
                            end
                        end
                    end
                    jcontainer.setSelected(false);  % de-select the menu
                end
            end
        end
    end

%% Remove a specific tree node's data
    function nodeRemovedFlag = removeDuplicateNode(thisIdx)
        nodeRemovedFlag = false;
        for idx = 1 : thisIdx-1
            if isequal(handles(idx),handles(thisIdx))
                levels(thisIdx) = [];
                parentIdx(thisIdx) = [];
                handles(thisIdx) = [];
                positions(thisIdx,:) = [];
                %sizes(thisIdx,:) = [];
                nodeRemovedFlag = true;
                return;
            end
        end
    end

%% Process optional args
    function processArgs(varargin)

        % Initialize
        invertFlag = false;
        listing = '';

        % Loop over all optional args
        while ~isempty(varargin) && ~isempty(handles)

            % Process the arg (and all its params)
            foundIdx = 1 : length(handles);
            if iscell(varargin{1}),  varargin{1} = varargin{1}{1};  end
            if ~isempty(varargin{1}) && varargin{1}(1)=='-'
                varargin{1}(1) = [];
            end
            switch lower(varargin{1})
                case 'not'
                    invertFlag = true;
                case 'position'
                    [varargin,foundIdx] = processPositionArgs(varargin{:});
                    if invertFlag,  foundIdx = ~foundIdx;  invertFlag = false;  end
                case 'size'
                    [varargin,foundIdx] = processSizeArgs(varargin{:});
                    if invertFlag,  foundIdx = ~foundIdx;  invertFlag = false;  end
                case 'class'
                    [varargin,foundIdx] = processClassArgs(varargin{:});
                    if invertFlag,  foundIdx = ~foundIdx;  invertFlag = false;  end
                case 'property'
                    [varargin,foundIdx] = processPropertyArgs(varargin{:});
                    if invertFlag,  foundIdx = ~foundIdx;  invertFlag = false;  end
                case 'depth'
                    [varargin,foundIdx] = processDepthArgs(varargin{:});
                    if invertFlag,  foundIdx = ~foundIdx;  invertFlag = false;  end
                case 'flat'
                    varargin = {'depth',0, varargin{min(2:end):end}};
                    [varargin,foundIdx] = processDepthArgs(varargin{:});
                    if invertFlag,  foundIdx = ~foundIdx;  invertFlag = false;  end
                case {'print','list'}
                    [varargin,listing] = processPrintArgs(varargin{:});
                case {'persist','nomenu','debug'}
                    % ignore - already handled in main function above
                otherwise
                    error('YMA:findjobj:IllegalOption',['Option ' num2str(varargin{1}) ' is not a valid option. Type ''help ' mfilename ''' for the full options list.']);
            end

            % If only parent-child pairs found
            foundIdx = find(foundIdx);
            if ~isempty(foundIdx) && isequal(parentIdx(foundIdx(2:2:end)),foundIdx(1:2:end))
                % Return just the children (the parent panels are uninteresting)
                foundIdx(1:2:end) = [];
            end

            % Filter the results
            selectedIdx = selectedIdx(foundIdx);
            handles   = handles(foundIdx);
            levels    = levels(foundIdx);
            parentIdx = parentIdx(foundIdx);
            positions = positions(foundIdx,:);

            % Remove this arg and proceed to the next one
            varargin(1) = [];

        end  % Loop over all args
    end

%% Process 'print' option
    function [varargin,listing] = processPrintArgs(varargin)
        if length(varargin)<2 || ischar(varargin{2})
            % No second arg given, so use the first available element
            listingContainer = handles(1);  %#ok - used in evalc below
        else
            % Get the element to print from the specified second arg
            if isnumeric(varargin{2}) && (varargin{2} == fix(varargin{2}))  % isinteger doesn't work on doubles...
                if (varargin{2} > 0) && (varargin{2} <= length(handles))
                    listingContainer = handles(varargin{2});  %#ok - used in evalc below
                elseif varargin{2} > 0
                    error('YMA:findjobj:IllegalPrintFilter','Print filter index %g > number of available elements (%d)',varargin{2},length(handles));
                else
                    error('YMA:findjobj:IllegalPrintFilter','Print filter must be a java handle or positive numeric index into handles');
                end
            elseif ismethod(varargin{2},'list')
                listingContainer = varargin{2};  %#ok - used in evalc below
            else
                error('YMA:findjobj:IllegalPrintFilter','Print filter must be a java handle or numeric index into handles');
            end
            varargin(2) = [];
        end

        % use evalc() to capture output into a Matlab variable
        %listing = evalc('listingContainer.list');

        % Better solution: loop over all handles and process them one by one
        listing = cell(length(handles),1);
        for componentIdx = 1 : length(handles)
            listing{componentIdx} = [repmat(' ',1,levels(componentIdx)) char(handles(componentIdx).toString)];
        end
    end

%% Process 'position' option
    function [varargin,foundIdx] = processPositionArgs(varargin)
        if length(varargin)>1
            positionFilter = varargin{2};
            %if (isjava(positionFilter) || iscom(positionFilter)) && ismethod(positionFilter,'getLocation')
            try  % try-catch is faster...
                % Java/COM object passed - get its position
                positionFilter = positionFilter.getLocation;
                filterXY = [positionFilter.getX, positionFilter.getY];
            catch
                if ~isscalar(positionFilter)
                    % position vector passed
                    if (length(positionFilter)>=2) && isnumeric(positionFilter)
                        % Remember that java coordinates start at top-left corner, Matlab coords start at bottom left...
                        %positionFilter = java.awt.Point(positionFilter(1), container.getHeight - positionFilter(2));
                        filterXY = [container.getX + positionFilter(1), container.getY + contentSize(2) - positionFilter(2)];

                        % Check for full Matlab position vector (x,y,w,h)
                        %if (length(positionFilter)==4)
                        %    varargin{end+1} = 'size';
                        %    varargin{end+1} = fix(positionFilter(3:4));
                        %end
                    else
                        error('YMA:findjobj:IllegalPositionFilter','Position filter must be a java UI component, or X,Y pair');
                    end
                elseif length(varargin)>2
                    % x,y passed as separate arg values
                    if isnumeric(positionFilter) && isnumeric(varargin{3})
                        % Remember that java coordinates start at top-left corner, Matlab coords start at bottom left...
                        %positionFilter = java.awt.Point(positionFilter, container.getHeight - varargin{3});
                        filterXY = [container.getX + positionFilter, container.getY + contentSize(2) - varargin{3}];
                        varargin(3) = [];
                    else
                        error('YMA:findjobj:IllegalPositionFilter','Position filter must be a java UI component, or X,Y pair');
                    end
                else
                    error('YMA:findjobj:IllegalPositionFilter','Position filter must be a java UI component, or X,Y pair');
                end
            end

            % Compute the required element positions in order to be eligible for a more detailed examination
            % Note: based on the following constraints: 0 <= abs(elementX-filterX) + abs(elementY+elementH-filterY) < 7
            baseDeltas = [positions(:,1)-filterXY(1), positions(:,2)-filterXY(2)];  % faster than repmat()...
            %baseHeight = - baseDeltas(:,2);% -abs(baseDeltas(:,1));
            %minHeight = baseHeight - 7;
            %maxHeight = baseHeight + 7;
            %foundIdx = ~arrayfun(@(b)(invoke(b,'contains',positionFilter)),handles);  % ARGH! - disallowed by Matlab!
            %foundIdx = repmat(false,1,length(handles));
            %foundIdx(length(handles)) = false;  % faster than repmat()...
            foundIdx = (abs(baseDeltas(:,1)) < 7) & (abs(baseDeltas(:,2)) < 7); % & (minHeight >= 0);
            %fi = find(foundIdx);
            %for componentIdx = 1 : length(fi)
                %foundIdx(componentIdx) = handles(componentIdx).getBounds.contains(positionFilter);

                % Search for a point no farther than 7 pixels away (prevents rounding errors)
                %foundIdx(componentIdx) = handles(componentIdx).getLocationOnScreen.distanceSq(positionFilter) < 50;  % fails for invisible components...

                %p = java.awt.Point(positions(componentIdx,1), positions(componentIdx,2) + handles(componentIdx).getHeight);
                %foundIdx(componentIdx) = p.distanceSq(positionFilter) < 50;

                %foundIdx(componentIdx) = sum(([baseDeltas(componentIdx,1),baseDeltas(componentIdx,2)+handles(componentIdx).getHeight]).^2) < 50;

                % Following is the fastest method found to date: only eligible elements are checked in detailed
            %    elementHeight = handles(fi(componentIdx)).getHeight;
            %    foundIdx(fi(componentIdx)) = elementHeight > minHeight(fi(componentIdx)) && ...
            %                                 elementHeight < maxHeight(fi(componentIdx));
                %disp([componentIdx,elementHeight,minHeight(fi(componentIdx)),maxHeight(fi(componentIdx)),foundIdx(fi(componentIdx))])
            %end

            varargin(2) = [];
        else
            foundIdx = [];
        end
    end

%% Process 'size' option
    function [varargin,foundIdx] = processSizeArgs(varargin)
        if length(varargin)>1
            sizeFilter = lower(varargin{2});
            %if (isjava(sizeFilter) || iscom(sizeFilter)) && ismethod(sizeFilter,'getSize')
            try  % try-catch is faster...
                % Java/COM object passed - get its size
                sizeFilter = sizeFilter.getSize;
                filterWidth  = sizeFilter.getWidth;
                filterHeight = sizeFilter.getHeight;
            catch
                if ~isscalar(sizeFilter)
                    % size vector passed
                    if (length(sizeFilter)>=2) && isnumeric(sizeFilter)
                        %sizeFilter = java.awt.Dimension(sizeFilter(1),sizeFilter(2));
                        filterWidth  = sizeFilter(1);
                        filterHeight = sizeFilter(2);
                    else
                        error('YMA:findjobj:IllegalSizeFilter','Size filter must be a java UI component, or W,H pair');
                    end
                elseif length(varargin)>2
                    % w,h passed as separate arg values
                    if isnumeric(sizeFilter) && isnumeric(varargin{3})
                        %sizeFilter = java.awt.Dimension(sizeFilter,varargin{3});
                        filterWidth  = sizeFilter;
                        filterHeight = varargin{3};
                        varargin(3) = [];
                    else
                        error('YMA:findjobj:IllegalSizeFilter','Size filter must be a java UI component, or W,H pair');
                    end
                else
                    error('YMA:findjobj:IllegalSizeFilter','Size filter must be a java UI component, or W,H pair');
                end
            end
            %foundIdx = ~arrayfun(@(b)(invoke(b,'contains',sizeFilter)),handles);  % ARGH! - disallowed by Matlab!
            foundIdx(length(handles)) = false;  % faster than repmat()...
            for componentIdx = 1 : length(handles)
                %foundIdx(componentIdx) = handles(componentIdx).getSize.equals(sizeFilter);
                % Allow a 2-pixel tollerance to account for non-integer pixel sizes
                foundIdx(componentIdx) = abs(handles(componentIdx).getWidth  - filterWidth)  <= 2 && ...  % faster than getSize.equals()
                                         abs(handles(componentIdx).getHeight - filterHeight) <= 2;
            end
            varargin(2) = [];
        else
            foundIdx = [];
        end
    end

%% Process 'class' option
    function [varargin,foundIdx] = processClassArgs(varargin)
        if length(varargin)>1
            classFilter = varargin{2};
            %if ismethod(classFilter,'getClass')
            try  % try-catch is faster...
                classFilter = char(classFilter.getClass);
            catch
                if ~ischar(classFilter)
                    error('YMA:findjobj:IllegalClassFilter','Class filter must be a java object, class or string');
                end
            end

            % Now convert all java classes to java.lang.Strings and compare to the requested filter string
            try
                foundIdx(length(handles)) = false;  % faster than repmat()...
                jClassFilter = java.lang.String(classFilter).toLowerCase;
                for componentIdx = 1 : length(handles)
                    % Note: JVM 1.5's String.contains() appears slightly slower and is available only since Matlab 7.2
                    foundIdx(componentIdx) = handles(componentIdx).getClass.toString.toLowerCase.indexOf(jClassFilter) >= 0;
                end
            catch
                % Simple processing: slower since it does extra processing within opaque.char()
                for componentIdx = 1 : length(handles)
                    % Note: using @toChar is faster but returns java String, not a Matlab char
                    foundIdx(componentIdx) = ~isempty(regexpi(char(handles(componentIdx).getClass),classFilter));
                end
            end

            varargin(2) = [];
        else
            foundIdx = [];
        end
    end

%% Process 'property' option
    function [varargin,foundIdx] = processPropertyArgs(varargin)
        if length(varargin)>1
            propertyName = varargin{2};
            if iscell(propertyName)
                if length(propertyName) == 2
                    propertyVal  = propertyName{2};
                    propertyName = propertyName{1};
                elseif length(propertyName) == 1
                    propertyName = propertyName{1};
                else
                    error('YMA:findjobj:IllegalPropertyFilter','Property filter must be a string (case insensitive name of property) or cell array {propName,propValue}');
                end
            end
            if ~ischar(propertyName)
                error('YMA:findjobj:IllegalPropertyFilter','Property filter must be a string (case insensitive name of property) or cell array {propName,propValue}');
            end
            propertyName = lower(propertyName);
            %foundIdx = arrayfun(@(h)isprop(h,propertyName),handles);  % ARGH! - disallowed by Matlab!
            foundIdx(length(handles)) = false;  % faster than repmat()...

            % Split processing depending on whether a specific property value was requested (ugly but faster...)
            if exist('propertyVal','var')
                for componentIdx = 1 : length(handles)
                    try
                        % Find out whether this element has the specified property
                        % Note: findprop() and its return value schema.prop are undocumented and unsupported!
                        prop = findprop(handles(componentIdx),propertyName);  % faster than isprop() & enables partial property names

                        % If found, compare it to the actual element's property value
                        foundIdx(componentIdx) = ~isempty(prop) && isequal(get(handles(componentIdx),prop.Name),propertyVal);
                    catch
                        % Some Java classes have a write-only property (like LabelPeer with 'Text'), so we end up here
                        % In these cases, simply assume that the property value doesn't match and continue
                        foundIdx(componentIdx) = false;
                    end
                end
            else
                for componentIdx = 1 : length(handles)
                    try
                        % Find out whether this element has the specified property
                        % Note: findprop() and its return value schema.prop are undocumented and unsupported!
                        foundIdx(componentIdx) = ~isempty(findprop(handles(componentIdx),propertyName));
                    catch
                        foundIdx(componentIdx) = false;
                    end
                end
            end
            varargin(2) = [];
        else
            foundIdx = [];
        end
    end

%% Process 'depth' option
    function [varargin,foundIdx] = processDepthArgs(varargin)
        if length(varargin)>1
            level = varargin{2};
            if ~isnumeric(level)
                error('YMA:findjobj:IllegalDepthFilter','Depth filter must be a number (=maximal element depth)');
            end
            foundIdx = (levels <= level);
            varargin(2) = [];
        else
            foundIdx = [];
        end
    end

%% Convert property data into a string
    function data = charizeData(data)
        if isa(data,'com.mathworks.hg.types.HGCallback')
            data = get(data,'Callback');
        end
        if ~ischar(data)
            newData = strtrim(evalc('disp(data)'));
            try
                newData = regexprep(newData,'  +',' ');
                newData = regexprep(newData,'Columns \d+ through \d+\s','');
                newData = regexprep(newData,'Column \d+\s','');
            catch
                %never mind...
            end
            if iscell(data)
                newData = ['{ ' newData ' }'];
            elseif isempty(data)
                newData = '';
            elseif isnumeric(data) || islogical(data) || any(ishandle(data)) || numel(data) > 1 %&& ~isscalar(data)
                newData = ['[' newData ']'];
            end
            data = newData;
        elseif ~isempty(data)
            data = ['''' data ''''];
        end
    end  % charizeData

%% Get callbacks table data
    function [cbData, cbHeaders, cbTableEnabled] = getCbsData(obj, stripStdCbsFlag)
        classHdl = classhandle(handle(obj));
        cbNames = get(classHdl.Events,'Name');
        if ~isempty(cbNames) && ~iscom(obj)  %only java-based please...
            cbNames = strcat(cbNames,'Callback');
        end
        propNames = get(classHdl.Properties,'Name');
        propCbIdx = [];
        if ~isempty(propNames)
            propCbIdx = find(~cellfun(@isempty,regexp(propNames,'(Fcn|Callback)$')));
            cbNames = unique([cbNames; propNames(propCbIdx)]);  %#ok logical is faster but less debuggable...
        end
        if ~isempty(cbNames)
            if stripStdCbsFlag
                cbNames = stripStdCbs(cbNames);
            end
            if iscell(cbNames)
                cbNames = sort(cbNames);
            end
            hgHandleFlag = 0;  try hgHandleFlag = ishghandle(obj); catch, end  %#ok
            try
                obj = handle(obj,'CallbackProperties');
            catch
                hgHandleFlag = 1;
            end
            if hgHandleFlag
                % HG handles don't allow CallbackProperties - search only for *Fcn
                cbNames = propNames(propCbIdx);
            end
            if iscom(obj)
                cbs = obj.eventlisteners;
                if ~isempty(cbs)
                    cbNamesRegistered = cbs(:,1);
                    cbData = setdiff(cbNames,cbNamesRegistered);
                    %cbData = charizeData(cbData);
                    if size(cbData,2) > size(cbData(1))
                        cbData = cbData';
                    end
                    cbData = [cbData, cellstr(repmat(' ',length(cbData),1))];
                    cbData = [cbData; cbs];
                    [sortedNames, sortedIdx] = sort(cbData(:,1));
                    sortedCbs = cellfun(@charizeData,cbData(sortedIdx,2),'un',0);
                    cbData = [sortedNames, sortedCbs];
                else
                    cbData = [cbNames, cellstr(repmat(' ',length(cbNames),1))];
                end
            elseif iscell(cbNames)
                cbNames = sort(cbNames);
                %cbData = [cbNames, get(obj,cbNames)'];
                cbData = cbNames;
                for idx = 1 : length(cbNames)
                    try
                        cbData{idx,2} = charizeData(get(obj,cbNames{idx}));
                    catch
                        cbData{idx,2} = '(callback value inaccessible)';
                    end
                end
            else  % only one event callback
                %cbData = {cbNames, get(obj,cbNames)'};
                %cbData{1,2} = charizeData(cbData{1,2});
                try
                    cbData = {cbNames, charizeData(get(obj,cbNames))};
                catch
                    cbData = {cbNames, '(callback value inaccessible)'};
                end
            end
            cbHeaders = {'Callback name','Callback value'};
            cbTableEnabled = true;
        else
            cbData = {'(no callbacks)'};
            cbHeaders = {'Callback name'};
            cbTableEnabled = false;
        end
    end  % getCbsData

%% Get relative (0.0-1.0) divider location
    function divLocation = getRalativeDivlocation(jDiv)
        divLocation = jDiv.getDividerLocation;
        if divLocation > 1  % i.e. [pixels]
            visibleRect = jDiv.getVisibleRect;
            if jDiv.getOrientation == 0  % vertical
                start = visibleRect.getY;
                extent = visibleRect.getHeight - start;
            else
                start = visibleRect.getX;
                extent = visibleRect.getWidth - start;
            end
            divLocation = (divLocation - start) / extent;
        end
    end  % getRalativeDivlocation

%% Try to set a treenode icon based on a container's icon
    function setTreeNodeIcon(treenode,container)
        try
            iconImage = [];
            iconImage = container.getIcon;
            if ~isempty(findprop(handle(iconImage),'Image'))  % get(iconImage,'Image') is easier but leaks memory...
                iconImage = iconImage.getImage;
            else
                a=b; %#ok cause an error
            end
        catch
            try
                iconImage = container.getIconImage;
            catch
                try
                    if ~isempty(iconImage)
                        ge = java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment;
                        gd = ge.getDefaultScreenDevice;
                        gc = gd.getDefaultConfiguration;
                        image = gc.createCompatibleImage(iconImage.getIconWidth, iconImage.getIconHeight);  % a BufferedImage object
                        g = image.createGraphics;
                        iconImage.paintIcon([], g, 0, 0);
                        g.dispose;
                        iconImage = image;
                    end
                catch
                    % never mind...
                end
            end
        end
        if ~isempty(iconImage)
            iconImage = setIconSize(iconImage);
            treenode.setIcon(iconImage);
        end
    end  % setTreeNodeIcon

%% Present the object hierarchy tree
    function presentObjectTree()
        import java.awt.*
        import javax.swing.*
        hTreeFig = findall(0,'tag','findjobjFig');
        iconpath = [matlabroot, '/toolbox/matlab/icons/'];
        cbHideStd = 0;  % Initial state of the cbHideStdCbs checkbox
        if isempty(hTreeFig)
            % Prepare the figure
            hTreeFig = figure('tag','findjobjFig','menuBar','none','toolBar','none','Name','FindJObj','NumberTitle','off','handleVisibility','off','IntegerHandle','off');
            figIcon = ImageIcon([iconpath 'tool_legend.gif']);
            drawnow;
            try
                mde = com.mathworks.mde.desk.MLDesktop.getInstance;
                jTreeFig = mde.getClient('FindJObj').getTopLevelAncestor;
                jTreeFig.setIcon(figIcon);
            catch
                jTreeFig = get(hTreeFig,'JavaFrame');
                jTreeFig.setFigureIcon(figIcon);
            end
            vsplitPaneLocation = 0.8;
            hsplitPaneLocation = 0.5;
        else
            % Remember cbHideStdCbs checkbox & dividers state for later
            userdata = get(hTreeFig, 'userdata');
            try cbHideStd = userdata.cbHideStdCbs.isSelected; catch, end  %#ok
            vsplitPaneLocation = getRalativeDivlocation(userdata.vsplitPane);
            hsplitPaneLocation = getRalativeDivlocation(userdata.hsplitPane);

            % Clear the figure and redraw
            clf(hTreeFig);
            figure(hTreeFig);   % bring to front
        end

        % Traverse all HG children, if root container was a HG handle
        if ishghandle(origContainer) %&& ~isequal(origContainer,container)
            traverseHGContainer(origContainer,0,0);
        end

        % Prepare the tree pane
        warning('off','MATLAB:uitreenode:MigratingFunction');  % R2008b compatibility
        tree_h = com.mathworks.hg.peer.UITreePeer;
        hasChildren = sum(allParents==1) > 1;
        icon = [iconpath 'upfolder.gif'];
        [rootName, rootTitle] = getNodeName(container);
        try
            root = uitreenode('v0', handle(container), rootName, icon, ~hasChildren);
        catch  % old matlab version don't have the 'v0' option
            root = uitreenode(handle(container), rootName, icon, ~hasChildren);
        end
        setTreeNodeIcon(root,container);  % constructor must accept a char icon unfortunately, so need to do this afterwards...
        if ~isempty(rootTitle)
            set(hTreeFig, 'Name',['FindJObj - ' char(rootTitle)]);
        end
        nodedata.idx = 1;
        nodedata.obj = container;
        set(root,'userdata',nodedata);
        root.setUserObject(container);
        setappdata(root,'childHandle',container);
        tree_h.setRoot(root);
        treePane = tree_h.getScrollPane;
        treePane.setMinimumSize(Dimension(50,50));
        jTreeObj = treePane.getViewport.getComponent(0);
        jTreeObj.setShowsRootHandles(true)
        jTreeObj.getSelectionModel.setSelectionMode(javax.swing.tree.TreeSelectionModel.DISCONTIGUOUS_TREE_SELECTION);
        %jTreeObj.setVisible(0);
        %jTreeObj.getCellRenderer.setLeafIcon([]);
        %jTreeObj.getCellRenderer.setOpenIcon(figIcon);
        %jTreeObj.getCellRenderer.setClosedIcon([]);
        treePanel = JPanel(BorderLayout);
        treePanel.add(treePane, BorderLayout.CENTER);
        progressBar = JProgressBar(0);
        progressBar.setMaximum(length(allHandles) + length(hg_handles));  % = # of all nodes
        treePanel.add(progressBar, BorderLayout.SOUTH);

        % Prepare the image pane
%disable for now, until we get it working...
%{
        try
            hFig = ancestor(origContainer,'figure');
            [cdata, cm] = getframe(hFig);  %#ok cm unused
            tempfname = [tempname '.png'];
            imwrite(cdata,tempfname);  % don't know how to pass directly to BufferedImage, so use disk...
            jImg = javax.imageio.ImageIO.read(java.io.File(tempfname));
            try delete(tempfname);  catch  end
            imgPanel = JPanel();
            leftPanel = JSplitPane(JSplitPane.VERTICAL_SPLIT, treePanel, imgPanel);
            leftPanel.setOneTouchExpandable(true);
            leftPanel.setContinuousLayout(true);
            leftPanel.setResizeWeight(0.8);
        catch
            leftPanel = treePanel;
        end
%}
        leftPanel = treePanel;

        % Prepare the inspector pane
        classNameLabel = JLabel(['      ' char(container.class)]);
        classNameLabel.setForeground(Color.red);
        updateNodeTooltip(container, classNameLabel);
        inspectorPanel = JPanel(BorderLayout);
        inspectorPanel.add(classNameLabel, BorderLayout.NORTH);
        % TODO: Maybe uncomment the following when we add the HG tree - in the meantime it's unused (java properties are un-groupable)
        %objReg = com.mathworks.services.ObjectRegistry.getLayoutRegistry;
        %toolBar = awtinvoke('com.mathworks.mlwidgets.inspector.PropertyView$ToolBarStyle','valueOf(Ljava.lang.String;)','GROUPTOOLBAR');
        %inspectorPane = com.mathworks.mlwidgets.inspector.PropertyView(objReg, toolBar);
        inspectorPane = com.mathworks.mlwidgets.inspector.PropertyView;
        identifiers = disableDbstopError;  %#ok "dbstop if error" causes inspect.m to croak due to a bug - so workaround
        inspectorPane.setObject(container);
        inspectorPane.setAutoUpdate(true);
        % TODO: Add property listeners
        % TODO: Display additional props
        inspectorTable = inspectorPane;
        while ~isa(inspectorTable,'javax.swing.JTable')
            inspectorTable = inspectorTable.getComponent(0);
        end
        toolTipText = 'hover mouse over the red classname above to see the full list of properties';
        inspectorTable.setToolTipText(toolTipText);
        jideTableUtils = [];
        try
            % Try JIDE features - see http://www.jidesoft.com/products/JIDE_Grids_Developer_Guide.pdf
            com.mathworks.mwswing.MJUtilities.initJIDE;
            jideTableUtils = eval('com.jidesoft.grid.TableUtils;');  % prevent JIDE alert by run-time (not load-time) evaluation
            jideTableUtils.autoResizeAllColumns(inspectorTable);
            inspectorTable.setRowAutoResizes(true);
            inspectorTable.getModel.setShowExpert(1);
        catch
            % JIDE is probably unavailable - never mind...
        end
        inspectorPanel.add(inspectorPane, BorderLayout.CENTER);
        % TODO: Add data update listeners

        % Prepare the callbacks pane
        callbacksPanel = JPanel(BorderLayout);
        classHdl = classhandle(handle(container));
        eventNames = get(classHdl.Events,'Name');
        if ~isempty(eventNames)
            cbNames = sort(strcat(eventNames,'Callback'));
            cbData = [cbNames, get(container,cbNames)'];
            cbTableEnabled = true;
        else
            cbData = {'(no callbacks)',''};
            cbTableEnabled = false;
        end
        cbHeaders = {'Callback name','Callback value'};
        try
            % Use JideTable if available on this system
            %callbacksTableModel = javax.swing.table.DefaultTableModel(cbData,cbHeaders);  %#ok
            %callbacksTable = eval('com.jidesoft.grid.PropertyTable(callbacksTableModel);');  % prevent JIDE alert by run-time (not load-time) evaluation
            callbacksTable = eval('com.jidesoft.grid.TreeTable(cbData,cbHeaders);');  % prevent JIDE alert by run-time (not load-time) evaluation
            callbacksTable.setRowAutoResizes(true);
            callbacksTable.setColumnAutoResizable(true);
            callbacksTable.setColumnResizable(true);
            jideTableUtils.autoResizeAllColumns(callbacksTable);
            callbacksTable.setTableHeader([]);  % hide the column headers since now we can resize columns with the gridline
            callbacksLabel = JLabel(' Callbacks:');  % The column headers are replaced with a header label
            %callbacksPanel.add(callbacksLabel, BorderLayout.NORTH);

            % Add checkbox to show/hide standard callbacks
            callbacksTopPanel = JPanel;
            callbacksTopPanel.setLayout(BoxLayout(callbacksTopPanel, BoxLayout.LINE_AXIS));
            callbacksTopPanel.add(callbacksLabel);
            callbacksTopPanel.add(Box.createHorizontalGlue);
            jcb = JCheckBox('Hide standard callbacks', cbHideStd);
            set(jcb, 'ActionPerformedCallback',@cbHideStdCbs_Callback, 'userdata',callbacksTable, 'tooltip','Hide standard Swing callbacks - only component-specific callbacks will be displayed');
            callbacksTopPanel.add(jcb);
            callbacksPanel.add(callbacksTopPanel, BorderLayout.NORTH);
        catch
            % Otherwise, use a standard Swing JTable (keep the headers to enable resizing)
            callbacksTable = JTable(cbData,cbHeaders);
        end
        cbToolTipText = 'Callbacks may be ''strings'', @funcHandle or {@funcHandle,arg1,...}';
        callbacksTable.setToolTipText(cbToolTipText);
        callbacksTable.setGridColor(inspectorTable.getGridColor);
        cbNameTextField = JTextField;
        cbNameTextField.setEditable(false);  % ensure that the callback names are not modified...
        cbNameCellEditor = DefaultCellEditor(cbNameTextField);
        cbNameCellEditor.setClickCountToStart(intmax);  % i.e, never enter edit mode...
        callbacksTable.getColumnModel.getColumn(0).setCellEditor(cbNameCellEditor);
        if ~cbTableEnabled
            callbacksTable.getColumnModel.getColumn(1).setCellEditor(cbNameCellEditor);
        end
        set(callbacksTable.getModel, 'TableChangedCallback',@tbCallbacksChanged, 'UserData',container);
        cbScrollPane = JScrollPane(callbacksTable);
        cbScrollPane.setVerticalScrollBarPolicy(cbScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED);
        callbacksPanel.add(cbScrollPane, BorderLayout.CENTER);
        callbacksPanel.setToolTipText(cbToolTipText);

        % Prepare the top-bottom JSplitPanes
        vsplitPane = JSplitPane(JSplitPane.VERTICAL_SPLIT, inspectorPanel, callbacksPanel);
        vsplitPane.setOneTouchExpandable(true);
        vsplitPane.setContinuousLayout(true);
        vsplitPane.setResizeWeight(0.8);

        % Prepare the left-right JSplitPane
        hsplitPane = JSplitPane(JSplitPane.HORIZONTAL_SPLIT, leftPanel, vsplitPane);
        hsplitPane.setOneTouchExpandable(true);
        hsplitPane.setContinuousLayout(true);
        hsplitPane.setResizeWeight(0.6);
        pos = getpixelposition(hTreeFig);

        % Prepare the bottom pane with all buttons
        lowerPanel = JPanel(FlowLayout);
        blogUrlLabel = '<a href="http://UndocumentedMatlab.com">Undocumented<br>Matlab.com</a>';
        jWebsite = createJButton(blogUrlLabel, @btWebsite_Callback, 'Visit the UndocumentedMatlab.com blog');
        jWebsite.setContentAreaFilled(0);
        lowerPanel.add(jWebsite);
        lowerPanel.add(createJButton('Refresh<br>tree',        {@btRefresh_Callback, origContainer, hTreeFig}, 'Rescan the component tree, from the root down'));
        lowerPanel.add(createJButton('Export to<br>workspace', {@btExport_Callback,  jTreeObj, classNameLabel}, 'Export the selected component handles to workspace variable findjobj_hdls'));
        lowerPanel.add(createJButton('Request<br>focus',       {@btFocus_Callback,   jTreeObj, root}, 'Set the focus on the first selected component'));
        lowerPanel.add(createJButton('Inspect<br>object',      {@btInspect_Callback, jTreeObj, root}, 'View the signature of all methods supported by the first selected component'));
        lowerPanel.add(createJButton('Check for<br>updates',       {@btCheckFex_Callback}, 'Check the MathWorks FileExchange for the latest version of FindJObj'));

        % Display everything on-screen
        globalPanel = JPanel(BorderLayout);
        globalPanel.add(hsplitPane, BorderLayout.CENTER);
        globalPanel.add(lowerPanel, BorderLayout.SOUTH);
        [obj, hcontainer] = javacomponent(globalPanel, [0,0,pos(3:4)], hTreeFig);
        set(hcontainer,'units','normalized');
        drawnow;
        hsplitPane.setDividerLocation(hsplitPaneLocation);  % this only works after the JSplitPane is displayed...
        vsplitPane.setDividerLocation(vsplitPaneLocation);  % this only works after the JSplitPane is displayed...
        %restoreDbstopError(identifiers);

        % Refresh & resize the screenshot thumbnail
%disable for now, until we get it working...
%{
        try
            hAx = axes('Parent',hTreeFig, 'units','pixels', 'position',[10,10,250,150], 'visible','off');
            axis(hAx,'image');
            image(cdata,'Parent',hAx);
            axis(hAx,'off');
            set(hAx,'UserData',cdata);
            set(imgPanel, 'ComponentResizedCallback',{@resizeImg, hAx}, 'UserData',lowerPanel);
            imgPanel.getGraphics.drawImage(jImg, 0, 0, []);
        catch
            % Never mind...
        end
%}
        % If all handles were selected (i.e., none were filtered) then only select the first
        if (length(selectedIdx) == length(allHandles)) && ~isempty(selectedIdx)
            selectedIdx = 1;
        end

        % Store handles for callback use
        userdata.handles = allHandles;
        userdata.levels  = allLevels;
        userdata.parents = allParents;
        userdata.hg_handles = hg_handles;
        userdata.hg_levels  = hg_levels;
        userdata.hg_parents = hg_parentIdx;
        userdata.initialIdx = selectedIdx;
        userdata.userSelected = false;  % Indicates the user has modified the initial selections
        userdata.inInit = true;
        userdata.jTree = jTreeObj;
        userdata.jTreePeer = tree_h;
        userdata.vsplitPane = vsplitPane;
        userdata.hsplitPane = hsplitPane;
        userdata.classNameLabel = classNameLabel;
        userdata.inspectorPane = inspectorPane;
        userdata.callbacksTable = callbacksTable;
        userdata.jideTableUtils = jideTableUtils;
        userdata.cbHideStdCbs = jcb;

        set(tree_h,        'userdata',userdata);
        set(hTreeFig,      'userdata',userdata);
        set(callbacksTable,'userdata',userdata);

        % Select the root node if requested
        % Note: we must do so here since all other nodes except the root are processed by expandNode
        if any(selectedIdx==1)
            tree_h.setSelectedNode(root);
        end

        % Set the initial cbHideStdCbs state
        try
            if jcb.isSelected
                drawnow;
                evd.getSource.isSelected = jcb.isSelected;
                cbHideStdCbs_Callback(jcb,evd);
            end
        catch
            % never mind...
        end

        % Set the callback functions
        set(tree_h, 'NodeExpandedCallback', {@nodeExpanded, tree_h});
        set(tree_h, 'NodeSelectedCallback', {@nodeSelected, tree_h});

        % Pre-expand all rows
        expandNode(progressBar, jTreeObj, tree_h, root, 0);
        %jTreeObj.setVisible(1);

        % Hide the progressbar now that we've finished expanding all rows
        try
            hsplitPane.getLeftComponent.setTopComponent(treePane);
        catch
            % Probably not a vSplitPane on the left...
            hsplitPane.setLeftComponent(treePane);
        end
        hsplitPane.setDividerLocation(hsplitPaneLocation);  % need to do it again...

        % Update userdata
        userdata.inInit = false;
        set(tree_h,  'userdata',userdata);
        set(hTreeFig,'userdata',userdata);

        % Set keyboard focus on the tree
        jTreeObj.requestFocus;
        drawnow;

        % Check for a newer version
        checkVersion();

        % Reset the last error
        lasterr('');
    end

%% Rresize image pane
    function resizeImg(varargin)  %#ok - unused (TODO: waiting for img placement fix...)
        try
            hPanel = varargin{1};
            hAx    = varargin{3};
            lowerPanel = get(hPanel,'UserData');
            newJPos = cell2mat(get(hPanel,{'X','Y','Width','Height'}));
            newMPos = [1,get(lowerPanel,'Height'),newJPos(3:4)];
            set(hAx, 'units','pixels', 'position',newMPos, 'Visible','on');
            uistack(hAx,'top');  % no good...
            set(hPanel,'Opaque','off');  % also no good...
        catch
            % Never mind...
            dispError
        end
        return;
    end

%% "dbstop if error" causes inspect.m to croak due to a bug - so workaround by temporarily disabling this dbstop
    function identifiers = disableDbstopError
        dbStat = dbstatus;
        idx = find(strcmp({dbStat.cond},'error'));
        identifiers = [dbStat(idx).identifier];
        if ~isempty(idx)
            dbclear if error;
            msgbox('''dbstop if error'' had to be disabled due to a Matlab bug that would have caused Matlab to crash.', 'FindJObj', 'warn');
        end
    end

%% Restore any previous "dbstop if error"
    function restoreDbstopError(identifiers)  %#ok
        for itemIdx = 1 : length(identifiers)
            eval(['dbstop if error ' identifiers{itemIdx}]);
        end
    end

%% Recursively expand all nodes (except toolbar/menubar) in startup
    function expandNode(progressBar, tree, tree_h, parentNode, parentRow)
        try
            if nargin < 5
                parentPath = javax.swing.tree.TreePath(parentNode.getPath);
                parentRow = tree.getRowForPath(parentPath);
            end
            tree.expandRow(parentRow);
            progressBar.setValue(progressBar.getValue+1);
            numChildren = parentNode.getChildCount;
            if (numChildren == 0)
                pause(0.0002);  % as short as possible...
                drawnow;
            end
            nodesToUnExpand = {'FigureMenuBar','MLMenuBar','MJToolBar','Box','uimenu','uitoolbar','ScrollBar'};
            numChildren = parentNode.getChildCount;
            for childIdx = 0 : numChildren-1
                childNode = parentNode.getChildAt(childIdx);

                % Expand child node if not leaf & not toolbar/menubar
                if childNode.isLeafNode
                    progressBar.setValue(progressBar.getValue+1);

                    % Pre-select the node based upon the user's FINDJOBJ filters
                    try
                        nodedata = get(childNode, 'userdata');
                        userdata = get(tree_h, 'userdata');
                        if ~ishghandle(nodedata.obj) && ~userdata.userSelected && any(userdata.initialIdx == nodedata.idx)
                            pause(0.0002);  % as short as possible...
                            drawnow;
                            if isempty(tree_h.getSelectedNodes)
                                tree_h.setSelectedNode(childNode);
                            else
                                newSelectedNodes = [tree_h.getSelectedNodes, childNode];
                                tree_h.setSelectedNodes(newSelectedNodes);
                            end
                        end
                    catch
                        % never mind...
                        dispError
                    end

                else
                    % Expand all non-leaves
                    expandNode(progressBar, tree, tree_h, childNode);

                    % Re-collapse toolbar/menubar etc., and also invisible containers
                    % Note: if we simply did nothing, progressbar would not have been updated...
                    try
                        childHandle = getappdata(childNode,'childHandle');  %=childNode.getUserObject
                        visible = childHandle.isVisible;
                    catch
                        visible = 1;
                    end
                    %if any(strcmp(childNode.getName,nodesToUnExpand))
                    %name = char(childNode.getName);
                    if any(cellfun(@(s)~isempty(strmatch(s,char(childNode.getName))),nodesToUnExpand)) || ~visible
                        childPath = javax.swing.tree.TreePath(childNode.getPath);
                        childRow = tree.getRowForPath(childPath);
                        tree.collapseRow(childRow);
                    end
                end
            end
        catch
            % never mind...
            dispError
        end
    end

%% Create utility buttons
    function hButton = createJButton(nameStr, handler, toolTipText)
        try
            jButton = javax.swing.JButton(['<html><body><center>' nameStr]);
            jButton.setCursor(java.awt.Cursor.getPredefinedCursor(java.awt.Cursor.HAND_CURSOR));
            jButton.setToolTipText(toolTipText);
            minSize = jButton.getMinimumSize;
            jButton.setMinimumSize(java.awt.Dimension(minSize.getWidth,35));
            hButton = handle(jButton,'CallbackProperties');
            set(hButton,'ActionPerformedCallback',handler);
        catch
            % Never mind...
        end
    end

%% Flash a component off/on for the specified duration
% note: starts with 'on'; if numTimes is odd then ends with 'on', otherwise with 'off'
    function flashComponent(jComps,delaySecs,numTimes)
        persistent redBorder redBorderPanels
        try
            if isempty(redBorder)  % reuse if possible
                redBorder = javax.swing.border.LineBorder(java.awt.Color.red,2,0);
            end
            for compIdx = 1 : length(jComps)
                try
                    oldBorder{compIdx} = jComps(compIdx).getBorder;  %#ok grow
                catch
                    oldBorder{compIdx} = [];  %#ok grow
                end
                isSettable(compIdx) = ismethod(jComps(compIdx),'setBorder');  %#ok grow
                if isSettable(compIdx)
                    try
                        % some components prevent border modification:
                        oldBorderFlag = jComps(compIdx).isBorderPainted;
                        if ~oldBorderFlag
                            jComps(compIdx).setBorderPainted(1);
                            isSettable(compIdx) = jComps(compIdx).isBorderPainted;  %#ok grow
                            jComps(compIdx).setBorderPainted(oldBorderFlag);
                        end
                    catch
                        % do nothing...
                    end
                end
                if compIdx > length(redBorderPanels)
                    redBorderPanels{compIdx} = javax.swing.JPanel;
                    redBorderPanels{compIdx}.setBorder(redBorder);
                    redBorderPanels{compIdx}.setOpaque(0);  % transparent interior, red border
                end
                try
                    redBorderPanels{compIdx}.setBounds(jComps(compIdx).getBounds);
                catch
                    % never mind - might be an HG handle
                end
            end
            for idx = 1 : 2*numTimes
                if idx>1,  pause(delaySecs);  end  % don't pause at start
                visible = mod(idx,2);
                for compIdx = 1 : length(jComps)
                    try
                        jComp = jComps(compIdx);

                        % Prevent Matlab crash (java buffer overflow...)
                        if jComp.isa('com.mathworks.mwswing.desk.DTSplitPane') || ...
                           jComp.isa('com.mathworks.mwswing.MJSplitPane')
                            continue;
                        end

                        % HG handles are highlighted by setting their 'Selected' property
                        if isa(jComp,'uimenu')
                            if visible
                                oldColor = get(jComp,'ForegroundColor');
                                setappdata(jComp,'findjobj_oldColor',oldColor);
                                set(jComp,'ForegroundColor','red');
                            else
                                oldColor = getappdata(jComp,'findjobj_oldColor');
                                set(jComp,'ForegroundColor',oldColor);
                                rmappdata(jComp,'ForegroundColor');
                            end
                            
                        elseif ishghandle(jComp)
                            if visible
                                set(jComp,'Selected','on');
                            else
                                set(jComp,'Selected','off');
                            end

                        else %if isjava(jComp)

                            jParent = jComps(compIdx).getParent;

                            % Most Java components allow modifying their borders
                            if isSettable(compIdx)
                                if visible
                                    jComp.setBorder(redBorder);
                                    try jComp.setBorderPainted(1); catch, end  %#ok
                                else %if ~isempty(oldBorder{compIdx})
                                    jComp.setBorder(oldBorder{compIdx});
                                end
                                jComp.repaint;

                            % The other Java components are highlighted by a transparent red-border
                            % panel that is placed on top of them in their parent's space
                            elseif ~isempty(jParent)
                                if visible
                                    jParent.add(redBorderPanels{compIdx});
                                    jParent.setComponentZOrder(redBorderPanels{compIdx},0);
                                else
                                    jParent.remove(redBorderPanels{compIdx});
                                end
                                jParent.repaint
                            end
                        end
                    catch
                        % never mind - try the next component (if any)
                    end
                end
                drawnow;
            end
        catch
            % never mind...
            dispError;
        end
        return;  % debug point
    end  % flashComponent

%% Select tree node
    function nodeSelected(src, evd, tree)  %#ok
        try
            %nodeHandle = evd.getCurrentNode.getUserObject;
            nodedata = get(evd.getCurrentNode,'userdata');
            nodeHandle = nodedata.obj;
            userdata = get(src,'userdata');
            if ~isempty(nodeHandle) && ~isempty(userdata)
                numSelections  = userdata.jTree.getSelectionCount;
                selectionPaths = userdata.jTree.getSelectionPaths;
                if (numSelections == 1)
                    % Indicate that the user has modified the initial selection (except if this was an initial auto-selected node)
                    if ~userdata.inInit
                        userdata.userSelected = true;
                        set(src,'userdata',userdata);
                    end

                    % Update the fully-qualified class name label
                    numInitialIdx = length(userdata.initialIdx);
                    thisHandle = nodeHandle;
                    try
                        if ~ishghandle(thisHandle)
                            thisHandle = java(nodeHandle);
                        end
                    catch
                        % never mind...
                    end
                    if ~userdata.inInit || (numInitialIdx == 1)
                        userdata.classNameLabel.setText(['      ' char(thisHandle.class)]);
                    else
                        userdata.classNameLabel.setText([' ' num2str(numInitialIdx) 'x handles (some handles hidden by unexpanded tree nodes)']);
                    end
                    if ishghandle(thisHandle)
                        userdata.classNameLabel.setText(userdata.classNameLabel.getText.concat(' (HG handle)'));
                    end
                    userdata.inspectorPane.dispose;  % remove props listeners - doesn't work...
                    updateNodeTooltip(nodeHandle, userdata.classNameLabel);

                    % Update the data properties inspector pane
                    % Note: we can't simply use the evd nodeHandle, because this node might have been DE-selected with only one other node left selected...
                    %nodeHandle = selectionPaths(1).getLastPathComponent.getUserObject;
                    nodedata = get(selectionPaths(1).getLastPathComponent,'userdata');
                    nodeHandle = nodedata.obj;
                    %identifiers = disableDbstopError;  % "dbstop if error" causes inspect.m to croak due to a bug - so workaround
                    userdata.inspectorPane.setObject(thisHandle);

                    % Update the callbacks table
                    try
                        stripStdCbsFlag = getappdata(userdata.callbacksTable,'hideStdCbs');
                        [cbData, cbHeaders, cbTableEnabled] = getCbsData(nodeHandle, stripStdCbsFlag);  %#ok cbTableEnabled unused
                        callbacksTableModel = javax.swing.table.DefaultTableModel(cbData,cbHeaders);
                        set(callbacksTableModel, 'TableChangedCallback',@tbCallbacksChanged, 'UserData',nodeHandle);
                        userdata.callbacksTable.setModel(callbacksTableModel)
                        userdata.callbacksTable.setRowAutoResizes(true);
                        userdata.jideTableUtils.autoResizeAllColumns(userdata.callbacksTable);
                    catch
                        % never mind...
                        dispError
                    end
                    pause(0.005);
                    drawnow;
                    %restoreDbstopError(identifiers);

                    % Highlight the selected object (if visible)
                    flashComponent(nodeHandle,0.2,3);

                elseif (numSelections > 1)  % Multiple selections

                    % Get the list of all selected nodes
                    jArray = javaArray('java.lang.Object', numSelections);
                    toolTipStr = '<html>';
                    sameClassFlag = true;
                    for idx = 1 : numSelections
                        %jArray(idx) = selectionPaths(idx).getLastPathComponent.getUserObject;
                        nodedata = get(selectionPaths(idx).getLastPathComponent,'userdata');
                        try
                            jArray(idx) = java(nodedata.obj);
                        catch
                            jArray(idx) = nodedata.obj;
                        end
                        toolTipStr = [toolTipStr '&nbsp;' jArray(idx).class '&nbsp;'];  %#ok grow
                        if (idx < numSelections),  toolTipStr = [toolTipStr '<br>'];  end  %#ok grow
                        if (idx > 1) && sameClassFlag && ~isequal(jArray(idx).getClass,jArray(1).getClass)
                            sameClassFlag = false;
                        end
                    end
                    toolTipStr = [toolTipStr '</html>'];

                    % Update the fully-qualified class name label
                    if sameClassFlag
                        classNameStr = jArray(1).class;
                    else
                        classNameStr = 'handle';
                    end
                    if all(ishghandle(jArray))
                        if strcmp(classNameStr,'handle')
                            classNameStr = 'HG handles';
                        else
                            classNameStr = [classNameStr ' (HG handles)'];
                        end
                    end
                    classNameStr = [' ' num2str(numSelections) 'x ' classNameStr];
                    userdata.classNameLabel.setText(classNameStr);
                    userdata.classNameLabel.setToolTipText(toolTipStr);

                    % Update the data properties inspector pane
                    %identifiers = disableDbstopError;  % "dbstop if error" causes inspect.m to croak due to a bug - so workaround
                    userdata.inspectorPane.getRegistry.setSelected(jArray, true);

                    % Update the callbacks table
                    try
                        % Get intersecting callback names & values
                        stripStdCbsFlag = getappdata(userdata.callbacksTable,'hideStdCbs');
                        [cbData, cbHeaders, cbTableEnabled] = getCbsData(jArray(1), stripStdCbsFlag);  %#ok cbHeaders & cbTableEnabled unused
                        if ~isempty(cbData)
                            cbNames = cbData(:,1);
                            for idx = 2 : length(jArray)
                                [cbData2, cbHeaders2] = getCbsData(jArray(idx), stripStdCbsFlag);  %#ok cbHeaders2 unused
                                if ~isempty(cbData2)
                                    newCbNames = cbData2(:,1);
                                    [cbNames, cbIdx, cb2Idx] = intersect(cbNames,newCbNames);  %#ok cb2Idx unused
                                    cbData = cbData(cbIdx,:);
                                    for cbIdx = 1 : length(cbNames)
                                        newIdx = find(strcmp(cbNames{cbIdx},newCbNames));
                                        if ~isequal(cbData2{newIdx,2}, cbData{cbIdx,2})
                                            cbData{cbIdx,2} = '<different values>';
                                        end
                                    end
                                else
                                    cbData = cbData([],:);  %=empty cell array
                                end
                                if isempty(cbData)
                                    break;
                                end
                            end
                        end
                        cbHeaders = {'Callback name','Callback value'};
                        callbacksTableModel = javax.swing.table.DefaultTableModel(cbData,cbHeaders);
                        set(callbacksTableModel, 'TableChangedCallback',@tbCallbacksChanged, 'UserData',jArray);
                        userdata.callbacksTable.setModel(callbacksTableModel)
                        userdata.callbacksTable.setRowAutoResizes(true);
                        userdata.jideTableUtils.autoResizeAllColumns(userdata.callbacksTable);
                    catch
                        % never mind...
                        dispError
                    end

                    pause(0.005);
                    drawnow;
                    %restoreDbstopError(identifiers);

                    % Highlight the selected objects (if visible)
                    flashComponent(jArray,0.2,3);
                end

                % TODO: Auto-highlight selected object (?)
                %nodeHandle.requestFocus;
            end
        catch
            dispError
        end
    end

%% IFF utility function for annonymous cellfun funcs
    function result = iff(test,trueVal,falseVal)  %#ok
        try
            if test
                result = trueVal;
            else
                result = falseVal;
            end
        catch
            result = false;
        end
    end

%% Get an HTML representation of the object's properties
    function dataFieldsStr = getPropsHtml(nodeHandle, dataFields)
        try
            % Get a text representation of the fieldnames & values
            dataFieldsStr = '';  % just in case the following croaks...
            if isempty(dataFields)
                return;
            end
            dataFieldsStr = evalc('disp(dataFields)');
            if dataFieldsStr(end)==char(10),  dataFieldsStr=dataFieldsStr(1:end-1);  end

            % Strip out callbacks
            dataFieldsStr = regexprep(dataFieldsStr,'^\s*\w*Callback(Data)?:[^\n]*$','','lineanchors');
            dataFieldsStr = regexprep(dataFieldsStr,'\n\n','\n');

            % HTMLize tooltip data
            % First, set the fields' font based on its read-write status
            nodeHandle = handle(nodeHandle);  % ensure this is a Matlab handle, not a java object
            fieldNames = fieldnames(dataFields);
            undefinedStr = '';
            for fieldIdx = 1 : length(fieldNames)
                thisFieldName = fieldNames{fieldIdx};
                accessFlags = get(findprop(nodeHandle,thisFieldName),'AccessFlags');
                if isfield(accessFlags,'PublicSet') && strcmp(accessFlags.PublicSet,'on')
                    % Bolden read/write fields
                    thisFieldFormat = ['<b>' thisFieldName '<b>:$2'];
                elseif ~isfield(accessFlags,'PublicSet')
                    % Undefined - probably a Matlab-defined field of com.mathworks.hg.peer.FigureFrameProxy...
                    thisFieldFormat = ['<font color="blue">' thisFieldName '</font>:$2'];
                    undefinedStr = ', <font color="blue">undefined</font>';
                else % PublicSet=='off'
                    % Gray-out & italicize any read-only fields
                    thisFieldFormat = ['<font color="#C0C0C0"><i>' thisFieldName '</i></font>:<font color="#C0C0C0"><i>$2<i></font>'];
                end
                dataFieldsStr = regexprep(dataFieldsStr, ['([\s\n])' thisFieldName ':([^\n]*)'], ['$1' thisFieldFormat]);
            end
        catch
            % never mind... - probably an ambiguous property name
            %dispError
        end

        % Method 1: simple <br> list
        %dataFieldsStr = strrep(dataFieldsStr,char(10),'&nbsp;<br>&nbsp;&nbsp;');

        % Method 2: 2x2-column <table>
        dataFieldsStr = regexprep(dataFieldsStr, '^\s*([^:]+:)([^\n]*)\n^\s*([^:]+:)([^\n]*)$', '<tr><td>&nbsp;$1</td><td>&nbsp;$2</td><td>&nbsp;&nbsp;&nbsp;&nbsp;$3</td><td>&nbsp;$4&nbsp;</td></tr>', 'lineanchors');
        dataFieldsStr = regexprep(dataFieldsStr, '^[^<]\s*([^:]+:)([^\n]*)$', '<tr><td>&nbsp;$1</td><td>&nbsp;$2</td><td>&nbsp;</td><td>&nbsp;</td></tr>', 'lineanchors');
        dataFieldsStr = ['(<b>modifiable</b>' undefinedStr ' &amp; <font color="#C0C0C0"><i>read-only</i></font> fields)<p>&nbsp;&nbsp;<table cellpadding="0" cellspacing="0">' dataFieldsStr '</table>'];
    end

%% Update tooltip string with a node's data
    function updateNodeTooltip(nodeHandle, uiObject)
        try
            toolTipStr = nodeHandle.class;
            dataFieldsStr = '';

            % Add HG annotation if relevant
            if ishghandle(nodeHandle)
                hgStr = ' HG Handle';
            else
                hgStr = '';
            end

            % Note: don't bulk-get because (1) not all properties are returned & (2) some properties cause a Java exception
            % Note2: the classhandle approach does not enable access to user-defined schema.props
            ch = classhandle(handle(nodeHandle));
            dataFields = [];
            [sortedNames, sortedIdx] = sort(get(ch.Properties,'Name'));
            for idx = 1 : length(sortedIdx)
                sp = ch.Properties(sortedIdx(idx));
                % TODO: some fields (see EOL comment below) generate a Java Exception from: com.mathworks.mlwidgets.inspector.PropertyRootNode$PropertyListener$1$1.run
                if strcmp(sp.AccessFlags.PublicGet,'on') % && ~any(strcmp(sp.Name,{'FixedColors','ListboxTop','Extent'}))
                    try
                        dataFields.(sp.Name) = get(nodeHandle, sp.Name);
                    catch
                        dataFields.(sp.Name) = '<font color="red">Error!</font>';
                    end
                else
                    dataFields.(sp.Name) = '(no public getter method)';
                end
            end
            dataFieldsStr = getPropsHtml(nodeHandle, dataFields);
        catch
            % Probably a non-HG java object
            try
                % Note: the bulk-get approach enables access to user-defined schema-props, but not to some original classhandle Properties...
                dataFields = get(nodeHandle);
                dataFieldsStr = getPropsHtml(nodeHandle, dataFields);
            catch
                % Probably a missing property getter implementation
                try
                    % Inform the user - bail out on error
                    err = lasterror;
                    dataFieldsStr = ['<p>' strrep(err.message, char(10), '<br>')];
                catch
                    % forget it...
                end
            end
        end

        % Set the object tooltip
        if ~isempty(dataFieldsStr)
            toolTipStr = ['<html>&nbsp;<b><u><font color="red">' char(toolTipStr) '</font></u></b>' hgStr ':&nbsp;' dataFieldsStr '</html>'];
        end
        uiObject.setToolTipText(toolTipStr);
    end

%% Expand tree node
    function nodeExpanded(src, evd, tree)  %#ok
        % tree = handle(src);
        % evdsrc = evd.getSource;
        evdnode = evd.getCurrentNode;

        if ~tree.isLoaded(evdnode)

            % Get the list of children TreeNodes
            childnodes = getChildrenNodes(tree, evdnode);

            % Add the HG sub-tree (unless already included in the first tree)
            childHandle = getappdata(evdnode.handle,'childHandle');  %=evdnode.getUserObject
            if evdnode.isRoot && ~isempty(hg_handles) && ~isequal(hg_handles(1).java, childHandle)
                childnodes = [childnodes, getChildrenNodes(tree, evdnode, true)];
            end

            % If we have a single child handle, wrap it within a javaArray for tree.add() to "swallow"
            if (length(childnodes) == 1)
                chnodes = childnodes;
                childnodes = javaArray('com.mathworks.hg.peer.UITreeNode', 1);
                childnodes(1) = java(chnodes);
            end

            % Add child nodes to the current node
            tree.add(evdnode, childnodes);
            tree.setLoaded(evdnode, true);
        end
    end

%% Get an icon image no larger than 16x16 pixels
    function iconImage = setIconSize(iconImage)
        try
            iconWidth  = iconImage.getWidth;
            iconHeight = iconImage.getHeight;
            if iconWidth > 16
                newHeight = fix(iconHeight * 16 / iconWidth);
                iconImage = iconImage.getScaledInstance(16,newHeight,iconImage.SCALE_SMOOTH);
            elseif iconHeight > 16
                newWidth = fix(iconWidth * 16 / iconHeight);
                iconImage = iconImage.getScaledInstance(newWidth,16,iconImage.SCALE_SMOOTH);
            end
        catch
            % never mind... - return original icon
        end
    end  % setIconSize

%% Get list of children nodes
    function nodes = getChildrenNodes(tree, parentNode, isRootHGNode)
        try
            iconpath = [matlabroot, '/toolbox/matlab/icons/'];
            nodes = handle([]);
            userdata = get(tree,'userData');
            hdls = userdata.handles;
            nodedata = get(parentNode,'userdata');
            if nargin < 3
                %isJavaNode = ~ishghandle(parentNode.getUserObject);
                isJavaNode = ~ishghandle(nodedata.obj);
                isRootHGNode = false;
            else
                isJavaNode = ~isRootHGNode;
            end

            % Search for this parent node in the list of all nodes
            parents = userdata.parents;
            nodeIdx = nodedata.idx;

            if isJavaNode && isempty(nodeIdx)  % Failback, in case userdata doesn't work for some reason...
                for hIdx = 1 : length(hdls)
                    %if isequal(handle(parentNode.getUserObject), hdls(hIdx))
                    if isequal(handle(nodedata.obj), hdls(hIdx))
                        nodeIdx = hIdx;
                        break;
                    end
                end
            end
            if ~isJavaNode
                if isRootHGNode  % =root HG node
                    thisChildHandle = userdata.hg_handles(1);
                    childName = getNodeName(thisChildHandle);
                    hasGrandChildren = any(parents==1);
                    icon = [];
                    if hasGrandChildren && length(hg_handles)>1
                        childName = childName.concat(' - HG root container');
                        icon = [iconpath 'figureicon.gif'];
                    end
                    try
                        nodes = uitreenode('v0', thisChildHandle, childName, icon, ~hasGrandChildren);
                    catch  % old matlab version don't have the 'v0' option
                        try
                            nodes = uitreenode(thisChildHandle, childName, icon, ~hasGrandChildren);
                        catch
                            % probably an invalid handle - ignore...
                        end
                    end

                    % Add the handler to the node's internal data
                    % Note: could also use 'userdata', but setUserObject() is recommended for TreeNodes
                    % Note2: however, setUserObject() sets a java *BeenAdapter object for HG handles instead of the required original class, so use setappdata
                    %nodes.setUserObject(thisChildHandle);
                    setappdata(nodes,'childHandle',thisChildHandle);
                    nodedata.idx = 1;
                    nodedata.obj = thisChildHandle;
                    set(nodes,'userdata',nodedata);
                    return;
                else  % non-root HG node
                    parents = userdata.hg_parents;
                    hdls    = userdata.hg_handles;
                end  % if isRootHGNode
            end  % if ~isJavaNode

            % If this node was found, get the list of its children
            if ~isempty(nodeIdx)
                %childIdx = setdiff(find(parents==nodeIdx),nodeIdx);
                childIdx = find(parents==nodeIdx);
                childIdx(childIdx==nodeIdx) = [];  % faster...
                numChildren = length(childIdx);
                for cIdx = 1 : numChildren
                    thisChildIdx = childIdx(cIdx);
                    thisChildHandle = hdls(thisChildIdx);
                    childName = getNodeName(thisChildHandle);
                    try
                        visible = thisChildHandle.Visible;
                        if visible
                            try visible = thisChildHandle.Width > 0; catch, end  %#ok
                        end
                        if ~visible
                            childName = ['<HTML><i><font color="gray">' char(childName) '</font></i></html>'];  %#ok grow
                        end
                    catch
                        % never mind...
                    end
                    hasGrandChildren = any(parents==thisChildIdx);
                    try
                        isaLabel = isa(thisChildHandle.java,'javax.swing.JLabel');
                    catch
                        isaLabel = 0;
                    end
                    if hasGrandChildren && ~any(strcmp(thisChildHandle.class,{'axes'}))
                        icon = [iconpath 'foldericon.gif'];
                    elseif isaLabel
                        icon = [iconpath 'tool_text.gif'];
                    else
                        icon = [];
                    end
                    try
                        nodes(cIdx) = uitreenode('v0', thisChildHandle, childName, icon, ~hasGrandChildren);
                    catch  % old matlab version don't have the 'v0' option
                        try
                            nodes(cIdx) = uitreenode(thisChildHandle, childName, icon, ~hasGrandChildren);
                        catch
                            % probably an invalid handle - ignore...
                        end
                    end

                    % Use existing object icon, if available
                    try
                        setTreeNodeIcon(nodes(cIdx),thisChildHandle);
                    catch
                        % probably an invalid handle - ignore...
                    end

                    % Pre-select the node based upon the user's FINDJOBJ filters
                    try
                        if isJavaNode && ~userdata.userSelected && any(userdata.initialIdx == thisChildIdx)
                            pause(0.0002);  % as short as possible...
                            drawnow;
                            if isempty(tree.getSelectedNodes)
                                tree.setSelectedNode(nodes(cIdx));
                            else
                                newSelectedNodes = [tree.getSelectedNodes, nodes(cIdx).java];
                                tree.setSelectedNodes(newSelectedNodes);
                            end
                        end
                    catch
                        % never mind...
                    end

                    % Add the handler to the node's internal data
                    % Note: could also use 'userdata', but setUserObject() is recommended for TreeNodes
                    % Note2: however, setUserObject() sets a java *BeenAdapter object for HG handles instead of the required original class, so use setappdata
                    % Note3: the following will error if invalid handle - ignore
                    try
                        if isJavaNode
                            thisChildHandle = thisChildHandle.java;
                        end
                        %nodes(cIdx).setUserObject(thisChildHandle);
                        setappdata(nodes(cIdx),'childHandle',thisChildHandle);
                        nodedata.idx = thisChildIdx;
                        nodedata.obj = thisChildHandle;
                        set(nodes(cIdx),'userdata',nodedata);
                    catch
                        % never mind (probably an invalid handle) - leave unchanged (like a leaf)
                    end
                end
            end
        catch
            % Never mind - leave unchanged (like a leaf)
            %error('YMA:findjobj:UnknownNodeType', 'Error expanding component tree node');
            dispError
        end
    end

%% Get a node's name
    function [nodeName, nodeTitle] = getNodeName(hndl)
        try
            % Initialize (just in case one of the succeding lines croaks)
            nodeName = '';
            if ~ismethod(hndl,'getClass')
                try
                    nodeName = hndl.class;
                catch
                    nodeName = hndl.type;  % last-ditch try...
                end
            else
                nodeName = hndl.getClass.getSimpleName;
            end

            % Strip away the package name, leaving only the regular classname
            if ~isempty(nodeName) && ischar(nodeName)
                nodeName = java.lang.String(nodeName);
                nodeName = nodeName.substring(nodeName.lastIndexOf('.')+1);
            end
            if (nodeName.length == 0)
                % fix case of anonymous internal classes, that do not have SimpleNames
                try
                    nodeName = hndl.getClass.getName;
                    nodeName = nodeName.substring(nodeName.lastIndexOf('.')+1);
                catch
                    % never mind - leave unchanged...
                end
            end

            % Get any unique identifying string (if available in one of several fields)
            labelsToCheck = {'label','title','text','string','displayname','toolTipText','TooltipString','actionCommand','name','Tag','style','UIClassID'};
            nodeTitle = '';
            strField = '';  %#ok - used for debugging
            while ((~isa(nodeTitle,'java.lang.String') && ~ischar(nodeTitle)) || isempty(nodeTitle)) && ~isempty(labelsToCheck)
                try
                    nodeTitle = get(hndl,labelsToCheck{1});
                    strField = labelsToCheck{1};  %#ok - used for debugging
                catch
                    % never mind - probably missing prop, so skip to next one
                end
                labelsToCheck(1) = [];
            end
            if length(nodeTitle) ~= numel(nodeTitle)
                % Multi-line - convert to a long single line
                nodeTitle = nodeTitle';
                nodeTitle = nodeTitle(:)';
            end
            extraStr = regexprep(nodeTitle,{sprintf('(.{25,%d}).*',min(25,length(nodeTitle)-1)),' +'},{'$1...',' '},'once');
            if ~isempty(extraStr)
                if ischar(extraStr)
                    nodeName = nodeName.concat(' (''').concat(extraStr).concat(''')');
                else
                    nodeName = nodeName.concat(' (').concat(num2str(extraStr)).concat(')');
                end
                %nodeName = nodeName.concat(strField);
            end
        catch
            % Never mind - use whatever we have so far
            %dispError
        end
    end

%% Strip standard Swing callbacks from a list of events
    function evNames = stripStdCbs(evNames)
        try
            stdEvents = {'AncestorAdded', 'AncestorMoved', 'AncestorRemoved', 'AncestorResized', ...
                         'CaretPositionChanged', 'ComponentAdded', 'ComponentRemoved', ...
                         'ComponentHidden', 'ComponentMoved', 'ComponentResized', 'ComponentShown', ...
                         'PropertyChange', 'FocusGained', 'FocusLost', ...
                         'HierarchyChanged', 'InputMethodTextChanged', ...
                         'KeyPressed', 'KeyReleased', 'KeyTyped', ...
                         'MouseClicked', 'MouseDragged', 'MouseEntered', 'MouseExited', ...
                         'MouseMoved', 'MousePressed', 'MouseReleased', 'MouseWheelMoved', ...
                         'VetoableChange'};
            stdEvents = [stdEvents, strcat(stdEvents,'Callback'), strcat(stdEvents,'Fcn')];
            evNames = setdiff(evNames,stdEvents)';
        catch
            % Never mind...
            dispError
        end
    end

%% Callback function for <Hide standard callbacks> checkbox
    function cbHideStdCbs_Callback(src, evd, varargin)  %#ok
        try
            % Store the current checkbox value for later use
            callbacksTable = get(src,'userdata');
            if evd.getSource.isSelected
                setappdata(callbacksTable,'hideStdCbs',1);
            else
                setappdata(callbacksTable,'hideStdCbs',[]);
            end

            % Rescan the current node
            userdata = get(callbacksTable,'userdata');
            ed.getCurrentNode = userdata.jTree.getSelectionModel.getSelectionPath.getLastPathComponent;
            nodeSelected(userdata.jTreePeer,ed,[]);
        catch
            % Never mind...
            dispError
        end
    end

%% Callback function for <UndocumentedMatlab.com> button
    function btWebsite_Callback(src, evd, varargin)  %#ok
        try
            web('http://UndocumentedMatlab.com/');
        catch
            % Never mind...
            dispError
        end
    end

%% Callback function for <Refresh data> button
    function btRefresh_Callback(src, evd, varargin)  %#ok
        try
            % Set cursor shape to hourglass until we're done
            hTreeFig = varargin{2};
            set(hTreeFig,'Pointer','watch');
            drawnow;
            object = varargin{1};

            % Re-invoke this utility to re-scan the container for all children
            findjobj(object);
        catch
            % Never mind...
        end

        % Restore default cursor shape
        set(hTreeFig,'Pointer','arrow');
    end

%% Callback function for <Export> button
    function btExport_Callback(src, evd, varargin)  %#ok
        try
            % Get the list of all selected nodes
            jTree = varargin{1};
            numSelections  = jTree.getSelectionCount;
            selectionPaths = jTree.getSelectionPaths;
            hdls = handle([]);
            for idx = 1 : numSelections
                %hdls(idx) = handle(selectionPaths(idx).getLastPathComponent.getUserObject);
                nodedata = get(selectionPaths(idx).getLastPathComponent,'userdata');
                hdls(idx) = handle(nodedata.obj);
            end

            % Assign the handles in the base workspace & inform user
            assignin('base','findjobj_hdls',hdls);
            classNameLabel = varargin{2};
            msg = ['Exported ' char(classNameLabel.getText.trim) ' to base workspace variable findjobj_hdls'];
            msgbox(msg,'FindJObj','help');
        catch
            % Never mind...
            dispError
        end
    end

%% Callback function for <Request focus> button
    function btFocus_Callback(src, evd, varargin)  %#ok
        try
            % Request focus for the specified object
            object = getTopSelectedObject(varargin{:});
            object.requestFocus;
        catch
            try
                object = object.java.getPeer.requestFocus;
                object.requestFocus;
            catch
                % Never mind...
                %dispError
            end
        end
    end

%% Callback function for <Inspect> button
    function btInspect_Callback(src, evd, varargin)  %#ok
        try
            % Inspect the specified object
            object = getTopSelectedObject(varargin{:});
            if isempty(which('uiinspect'))

                % If the user has not indicated NOT to be informed about UIInspect
                if ~ispref('FindJObj','dontCheckUIInspect')

                    % Ask the user whether to download UIINSPECT (YES, no, no & don't ask again)
                    answer = questdlg({'The object inspector requires UIINSPECT from the MathWorks File Exchange. UIINSPECT was created by Yair Altman, like this FindJObj utility.','','Download & install UIINSPECT?'},'UIInspect','Yes','No','No & never ask again','Yes');
                    switch answer
                        case 'Yes'  % => Yes: download & install
                            try
                                % Download UIINSPECT
                                baseUrl = 'http://www.mathworks.com/matlabcentral/fileexchange/17935';
                                fileUrl = [baseUrl '?controller=file_infos&download=true'];
                                file = urlread(fileUrl);
                                file = regexprep(file,[char(13),char(10)],'\n');  %convert to OS-dependent EOL

                                % Install...
                                newPath = fullfile(fileparts(which(mfilename)),'uiinspect.m');
                                fid = fopen(newPath,'wt');
                                fprintf(fid,'%s',file);
                                fclose(fid);
                            catch
                                % Error downloading: inform the user
                                msgbox(['Error in downloading: ' lasterr], 'UIInspect', 'warn');
                                web(baseUrl);
                            end

                            % ...and now run it...
                            %pause(0.1); 
                            drawnow;
                            dummy = which('uiinspect');  %#ok used only to load into memory
                            uiinspect(object);
                            return;

                        case 'No & never ask again'   % => No & don't ask again
                            setpref('FindJObj','dontCheckUIInspect',1);

                        otherwise
                            % forget it...
                    end
                end
                drawnow;

                % No UIINSPECT available - run the good-ol' METHODSVIEW()...
                methodsview(object);
            else
                uiinspect(object);
            end
        catch
            try
                if isjava(object)
                    methodsview(object)
                else
                    methodsview(object.java);
                end
            catch
                % Never mind...
                dispError
            end
        end
    end

%% Callback function for <Check for updates> button
    function btCheckFex_Callback(src, evd, varargin)  %#ok
        try
            % Check the FileExchange for the latest version
            web('http://www.mathworks.com/matlabcentral/fileexchange/loadFile.do?objectId=14317');
        catch
            % Never mind...
            dispError
        end
    end

%% Check for existence of a newer version
    function checkVersion()
        try
            % If the user has not indicated NOT to be informed
            if ~ispref('FindJObj','dontCheckNewerVersion')

                % Get the latest version date from the File Exchange webpage
                baseUrl = 'http://www.mathworks.com/matlabcentral/fileexchange/';
                webUrl = [baseUrl '14317'];  % 'loadFile.do?objectId=14317'];
                webPage = urlread(webUrl);
                modIdx = strfind(webPage,'>Updates<');
                if ~isempty(modIdx)
                    webPage = webPage(modIdx:end);
                    % Note: regexp hangs if substr not found, so use strfind instead...
                    %latestWebVersion = regexprep(webPage,'.*?>(20[\d-]+)</td>.*','$1');
                    dateIdx = strfind(webPage,'class="date">');
                    if ~isempty(dateIdx)
                        latestDate = webPage(dateIdx(end)+13 : dateIdx(end)+23);
                        try
                            startIdx = dateIdx(end)+27;
                            descStartIdx = startIdx + strfind(webPage(startIdx:startIdx+999),'<td>');
                            descEndIdx   = startIdx + strfind(webPage(startIdx:startIdx+999),'</td>');
                            descStr = webPage(descStartIdx(1)+3 : descEndIdx(1)-2);
                        catch
                            descStr = '';
                        end

                        % Get this file's latest date
                        thisFileName = which(mfilename);  %#ok
                        try
                            thisFileData = dir(thisFileName);
                            try
                                thisFileDatenum = thisFileData.datenum;
                            catch  % old ML versions...
                                thisFileDatenum = datenum(thisFileData.date);
                            end
                        catch
                            thisFileText = evalc('type(thisFileName)');
                            thisFileLatestDate = regexprep(thisFileText,'.*Change log:[\s%]+([\d-]+).*','$1');
                            thisFileDatenum = datenum(thisFileLatestDate,'yyyy-mm-dd');
                        end

                        % If there's a newer version on the File Exchange webpage (allow 2 days grace period)
                        if (thisFileDatenum < datenum(latestDate,'dd mmm yyyy')-2)

                            % Ask the user whether to download the newer version (YES, no, no & don't ask again)
                            msg = {['A newer version (' latestDate ') of FindJObj is available on the MathWorks File Exchange:'], '', ...
                                   ['\color{blue}' descStr '\color{black}'], '', ...
                                   'Download & install the new version?'};
                            createStruct.Interpreter = 'tex';
                            createStruct.Default = 'Yes';
                            answer = questdlg(msg,'FindJObj','Yes','No','No & never ask again',createStruct);
                            switch answer
                                case 'Yes'  % => Yes: download & install newer file
                                    try
                                        %fileUrl = [baseUrl '/download.do?objectId=14317&fn=findjobj&fe=.m'];
                                        fileUrl = [baseUrl '/14317?controller=file_infos&download=true'];
                                        file = urlread(fileUrl);
                                        file = regexprep(file,[char(13),char(10)],'\n');  %convert to OS-dependent EOL
                                        fid = fopen(thisFileName,'wt');
                                        fprintf(fid,'%s',file);
                                        fclose(fid);
                                    catch
                                        % Error downloading: inform the user
                                        msgbox(['Error in downloading: ' lasterr], 'FindJObj', 'warn');
                                        web(webUrl);
                                    end
                                case 'No & never ask again'   % => No & don't ask again
                                    setpref('FindJObj','dontCheckNewerVersion',1);
                                otherwise
                                    % forget it...
                            end
                        end
                    end
                else
                    % Maybe webpage not fully loaded or changed format - bail out...
                end
            end
        catch
            % Never mind...
        end
    end

%% Get the first selected object (might not be the top one - depends on selection order)
    function object = getTopSelectedObject(jTree, root)
        try
            object = [];
            numSelections  = jTree.getSelectionCount;
            if numSelections > 0
                % Get the first object specified
                %object = jTree.getSelectionPath.getLastPathComponent.getUserObject;
                nodedata = get(jTree.getSelectionPath.getLastPathComponent,'userdata');
            else
                % Get the root object (container)
                %object = root.getUserObject;
                nodedata = get(root,'userdata');
            end
            object = nodedata.obj;
        catch
            % Never mind...
            dispError
        end
    end

%% Update component callback upon callbacksTable data change
    function tbCallbacksChanged(src, evd)
        try
            % exit if invalid handle or already in Callback
            if ~ishandle(src) || ~isempty(getappdata(src,'inCallback')) % || length(dbstack)>1  %exit also if not called from user action
                return;
            end
            setappdata(src,'inCallback',1);  % used to prevent endless recursion
            
            % Update the object's callback with the modified value
            modifiedColIdx = evd.getColumn;
            modifiedRowIdx = evd.getFirstRow;
            if modifiedColIdx==1 && modifiedRowIdx>=0  %sanity check - should always be true
                table = evd.getSource;
                object = get(src,'userdata');
                cbName = strtrim(table.getValueAt(modifiedRowIdx,0));
                try
                    cbValue = strtrim(char(table.getValueAt(modifiedRowIdx,1)));
                    if ~isempty(cbValue) && ismember(cbValue(1),'{[@''')
                        cbValue = eval(cbValue);
                    end
                    if (~ischar(cbValue) && ~isa(cbValue, 'function_handle') && (iscom(object(1)) || iscell(cbValue)))
                        revertCbTableModification(table, modifiedRowIdx, modifiedColIdx, cbName, object, '');
                    else
                        for objIdx = 1 : length(object)
                            if ~iscom(object(objIdx))
                                set(object(objIdx), cbName, cbValue);
                            else
                                cbs = object(objIdx).eventlisteners;
                                if ~isempty(cbs)
                                    cbs = cbs(strcmpi(cbs(:,1),cbName),:);
                                    object(objIdx).unregisterevent(cbs);
                                end
                                if ~isempty(cbValue)
                                    object(objIdx).registerevent({cbName, cbValue});
                                end
                            end
                        end
                    end
                catch
                    revertCbTableModification(table, modifiedRowIdx, modifiedColIdx, cbName, object, lasterr)
                end
            end
        catch
            % never mind...
        end
        setappdata(src,'inCallback',[]);  % used to prevent endless recursion
    end

%% Revert Callback table modification
    function revertCbTableModification(table, modifiedRowIdx, modifiedColIdx, cbName, object, errMsg)  %#ok
        try
            % Display a notification MsgBox
            msg = 'Callbacks must be a ''string'', or a @function handle';
            if ~iscom(object(1)),  msg = [msg ' or a {@func,args...} construct'];  end
            if ~isempty(errMsg),  msg = {errMsg, '', msg};  end
            msgbox(msg, ['Error setting ' cbName ' callback'], 'warn');
            
            % Revert to the current value
            curValue = '';
            try
                if ~iscom(object(1))
                    curValue = charizeData(get(object(1),cbName));
                else
                    cbs = object(1).eventlisteners;
                    if ~isempty(cbs)
                        cbs = cbs(strcmpi(cbs(:,1),cbName),:);
                        curValue = charizeData(cbs(1,2));
                    end
                end
            catch
                % never mind... - clear the current value
            end
            table.setValueAt(curValue, modifiedRowIdx, modifiedColIdx);
        catch
            % never mind...
        end
    end  % revertCbTableModification

%% Traverse an HG container hierarchy and extract the HG elements within
    function traverseHGContainer(hcontainer,level,parent)
       try
           % Record the data for this node
           thisIdx = length(hg_levels) + 1;
           hg_levels(thisIdx) = level;
           hg_parentIdx(thisIdx) = parent;
           hg_handles(thisIdx) = handle(hcontainer);
           parentId = length(hg_parentIdx);

           % Now recursively process all this node's children (if any)
           %if ishghandle(hcontainer)
           try  % try-catch is faster than checking ishghandle(hcontainer)...
               allChildren = allchild(handle(hcontainer));
               for childIdx = 1 : length(allChildren)
                   traverseHGContainer(allChildren(childIdx),level+1,parentId);
               end
           catch
               % do nothing - probably not a container
               %dispError
           end

           % TODO: Add axis (plot) component handles
       catch
           % forget it...
       end
    end

%% Debuggable "quiet" error-handling
    function dispError
        err = lasterror;
        msg = err.message;
        for idx = 1 : length(err.stack)
            filename = err.stack(idx).file;
            if ~isempty(regexpi(filename,mfilename))
                funcname = err.stack(idx).name;
                line = num2str(err.stack(idx).line);
                msg = [msg ' at <a href="matlab:opentoline(''' filename ''',' line ');">' funcname ' line #' line '</a>']; %#ok grow
                break;
            end
        end
        disp(msg);
        return;  % debug point
    end

end  % FINDJOBJ


%% TODO TODO TODO
%{
- Enh: Improve performance - esp. expandNode() (performance solved in non-nteractive mode)
- Enh: Add property listeners - same problem in MathWork's inspect.m
- Enh: Display additional properties - same problem in MathWork's inspect.m
- Enh: Add axis (plot, Graphics) component handles
- Enh: Group callbacks according to the first word (up to 2nd cap letter)
- Enh: Add figure thumbnail image below the java tree (& indicate corresponding jObject when selected)
- Enh: scroll initially-selected node into view (problem because treenode has no pixel location)
- Fix: java exceptions when getting some fields (com.mathworks.mlwidgets.inspector.PropertyRootNode$PropertyListener$1$1.run)
- Fix: use EDT if available (especially in flashComponent)
%}