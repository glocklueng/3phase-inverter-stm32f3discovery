function arm_sin_cos_f32(block)
% function wrapper
setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts = 1;
  block.NumOutputPorts = 2;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(1).SamplingMode     = 'sample';
  block.OutputPort(1).DatatypeID      = 1;
  
    block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(2).SamplingMode     = 'sample';
  block.OutputPort(2).DatatypeID      = 1;
  
  block.InputPort(1).Dimensions       = 1;
  block.InputPort(1).SamplingMode     = 'sample';
  block.InputPort(1).DatatypeID      = 1;   
  %% Set block sample time to [-1 0]
  block.SampleTimes = [-1 0];
  
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
%   block.Dwork(1).Dimensions      = 1;
%   block.Dwork(1).DatatypeID      = 3;
%   block.Dwork(1).Complexity      = 'Real';
%   block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
%% Initialize Dwork
 
  
%endfunction

function Output(block)
    block.OutputPort(1).Data = single(sin(2*pi/360*block.InputPort(1).Data(1)));
    block.OutputPort(2).Data = single(cos(2*pi/360*block.InputPort(1).Data(1)));
  
%endfunction

function Update(block)
    % no state
%endfunction

