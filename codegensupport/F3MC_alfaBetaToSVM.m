function F3MC_alfaBetaToSVM(block)
% this function has only one paramter:
%    fs     -   sampling frequency [Hz]
setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts = 2;
  block.NumOutputPorts = 0;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  %vše float
%   block.OutputPort(1).Dimensions       = 3;
%   block.OutputPort(1).SamplingMode     = 'sample';
%   block.OutputPort(1).DatatypeID      = 1;
%   
%   block.OutputPort(2).Dimensions       = 1;
%   block.OutputPort(2).SamplingMode     = 'sample';
%   block.OutputPort(2).DatatypeID      = 1;
%   
%   block.OutputPort(3).Dimensions       = 1;
%   block.OutputPort(3).SamplingMode     = 'sample';
%   block.OutputPort(3).DatatypeID      = 1;
%   
%   block.OutputPort(4).Dimensions       = 1;
%   block.OutputPort(4).SamplingMode     = 'sample';
%   block.OutputPort(4).DatatypeID      = 1;
  
   block.InputPort(1).Dimensions       = 1;
   block.InputPort(1).SamplingMode     = 'sample';
   block.InputPort(1).DatatypeID      = 1;
   
   block.InputPort(2).Dimensions       = 1;
   block.InputPort(2).SamplingMode     = 'sample';
   block.InputPort(2).DatatypeID      = 1;
  
   % Register the parameters.
  block.NumDialogPrms     = 1;
  %block.DialogPrm(1).Name = 'paramSampleRate';
  block.DialogPrmsTunable = {'Tunable'};
  
  
   
  %% Set block sample time to [-1 0]
  %% Set block sample time to [0.1 0]
  fs = block.DialogPrm(1).Data(1);
  block.SampleTimes = [1/fs 0];
  %block.WriteRTWParam('matrix', 'paramSampleRate', fs);
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Update',                  @Update);
  block.RegBlockMethod('WriteRTW',                @WriteRTW);
  
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
    global F3MC_alfa;
    global F3MC_beta;
    F3MC_alfa = single(block.InputPort(1).Data(1));
    F3MC_beta = single(block.InputPort(2).Data(1));
%endfunction

function Update(block)
    % no state
%endfunction

function WriteRTW(block)
  fs = sprintf('%f', block.DialogPrm(1).Data(1));
  block.WriteRTWParam('string', 'paramSampleRate', fs);
%endfunction

