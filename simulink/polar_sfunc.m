function movepoint_sfunc(block)

  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;
  
  % Setup port properties to be inherited or dynamic
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override input port properties
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(1).Dimensions        = 3;
  block.InputPort(1).DirectFeedthrough = true;

  % Override output port properties
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(1).Dimensions       = 4;

  block.NumDialogPrms     = 0;

  % Specify if Accelerator should use TLC or call back into 
  % M-file
  block.SetAccelRunOnTLC(false);
  
  block.SimStateCompliance = 'DefaultSimState';
  
  %block.RegBlockMethod('CheckParameters', @CheckPrms);

  %block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
  %block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
  %block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  %block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  %block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  %block.RegBlockMethod('SetInputPortComplexSignal', @SetInpPortComplexSig);
  %block.RegBlockMethod('SetOutputPortComplexSignal', @SetOutPortComplexSig);

  %% PostPropagationSetup:
  %%   Functionality    : Setup work areas and state variables. Can
  %%                      also register run-time methods here
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  %% Register methods called at run-time
  %% -----------------------------------------------------------------
  
  %% 
  %% ProcessParameters:
  %%   Functionality    : Called in order to allow update of run-time
  %%                      parameters
  %block.RegBlockMethod('ProcessParameters', @ProcessPrms);

  %% 
  %% InitializeConditions:
  %%   Functionality    : Called in order to initialize state and work
  %%                      area values
  %block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  %% 
  %% Start:
  %%   Functionality    : Called in order to initialize state and work
  %%                      area values
  block.RegBlockMethod('Start', @Start);

  %% 
  %% Outputs:
  %%   Functionality    : Called to generate block outputs in
  %%                      simulation step
  block.RegBlockMethod('Outputs', @Outputs);

  %% 
  %% Update:
  %%   Functionality    : Called to update discrete states
  %%                      during simulation step
  %%   C-Mex counterpart: mdlUpdate
  %%
  %block.RegBlockMethod('Update', @Update);

  %% 
  %% Derivatives:
  %%   Functionality    : Called to update derivatives of
  %%                      continuous states during simulation step
  %%   C-Mex counterpart: mdlDerivatives
  %%
  %block.RegBlockMethod('Derivatives', @Derivatives);
  
  %% 
  %% Projection:
  %%   Functionality    : Called to update projections during 
  %%                      simulation step
  %%   C-Mex counterpart: mdlProjections
  %%
  %block.RegBlockMethod('Projection', @Projection);
  
  %% 
  %% SimStatusChange:
  %%   Functionality    : Called when simulation goes to pause mode
  %%                      or continnues from pause mode
  %%   C-Mex counterpart: mdlSimStatusChange
  %%
  %block.RegBlockMethod('SimStatusChange', @SimStatusChange);
  
  %% 
  %% Terminate:
  %%   Functionality    : Called at the end of simulation for cleanup
  %%   C-Mex counterpart: mdlTerminate
  %%
  %block.RegBlockMethod('Terminate', @Terminate);

  %block.RegBlockMethod('WriteRTW', @WriteRTW);
end

function DoPostPropSetup(block)
    block.NumDworks = 1;
    block.Dwork(1).Name = 'direction';
    block.Dwork(1).Dimensions = 1;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = false;
end

function Start(block)
    block.Dwork(1).Data = 0;
end

function Outputs(block)
    X = block.InputPort(1).Data;

    x = X(1); y = X(2); theta = X(3);

    d = sqrt(x^2 + y^2);

    if block.Dwork(1).Data == 0
        beta = -atan2(-y, -x);
        alpha = -theta - beta;
        fprintf('alpha %f, beta %f\n', alpha, beta);
        % first time in simulation, choose the direction of travel
        if (alpha > pi/2) || (alpha < -pi/2)
            fprintf('going backwards\n');
            block.Dwork(1).Data = -1;
        else
            fprintf('going forwards\n');
            block.Dwork(1).Data = 1;
        end
    elseif block.Dwork(1).Data == -1
        beta = -atan2(y, x);
        alpha = -theta - beta;
    else
        beta = -atan2(-y, -x);
        alpha = -theta - beta;
    end
    if alpha > pi/2
        alpha = pi/2;
    end
    if alpha < -pi/2
        alpha = -pi/2;
    end

    block.OutputPort(1).Data = [d alpha beta block.Dwork(1).Data];
end
