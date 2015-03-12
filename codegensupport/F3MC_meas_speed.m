function F3MC_meas_speed(block)
% function wrapper
setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts = 0;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  %vše float
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(1).SamplingMode     = 'sample';
  block.OutputPort(1).DatatypeID      = 1;
  
 
  
%   block.InputPort(1).Dimensions       = 0;
%   block.InputPort(1).SamplingMode     = 'sample';
%   block.InputPort(1).DatatypeID      = 1;
  
  block.NumDialogPrms     = 1;
  Ts = block.DialogPrm(1).Data(1);
  block.SampleTimes = [Ts 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Update',                  @Update);  
  
%endfunction

function DoPostPropSetup(block)
  %% Setup Dwork
  block.NumDworks = 0;
%   block.Dwork(1).Name = 'x0'; 
%   block.Dwork(1).Dimensions      = 5;
%   block.Dwork(1).DatatypeID      = 1;
%   block.Dwork(1).Complexity      = 'Real';
%   block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
%% Initialize Dwork
  
%endfunction

function Output(block)
    global F3MC_omega;
    block.OutputPort(1).Data = single(F3MC_omega);
%     block.OutputPort(1).Data = single([ 0 ; 0 ; 0 ]);
%     block.OutputPort(2).Data = single([ 12 ]);
%     block.OutputPort(3).Data = single([ 0 ]);
%     block.OutputPort(4).Data = single([ 3000*pi/30 ]);
    
%endfunction

function Update(block)
    % no state
%endfunction

