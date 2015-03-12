function F3MC_measurement(block)
% function wrapper
setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts = 0;
  block.NumOutputPorts = 3;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  %vše float
  block.OutputPort(1).Dimensions       = 3;
  block.OutputPort(1).SamplingMode     = 'sample';
  block.OutputPort(1).DatatypeID      = 1;
  
  block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(2).SamplingMode     = 'sample';
  block.OutputPort(2).DatatypeID      = 1;
  
  block.OutputPort(3).Dimensions       = 1;
  block.OutputPort(3).SamplingMode     = 'sample';
  block.OutputPort(3).DatatypeID      = 1;
  
%   block.OutputPort(4).Dimensions       = 1;
%   block.OutputPort(4).SamplingMode     = 'sample';
%   block.OutputPort(4).DatatypeID      = 1;
  
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
  block.NumDworks = 1;
  block.Dwork(1).Name = 'x0'; 
  block.Dwork(1).Dimensions      = 5;
  block.Dwork(1).DatatypeID      = 1;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
%% Initialize Dwork
 block.Dwork(1).Data = single([ 0 0 0 0 15 ]);
  
%endfunction

function Output(block)
    ialpha = block.Dwork(1).Data(1);
    ibeta = block.Dwork(1).Data(2);
    theta = block.Dwork(1).Data(3);
    omega = block.Dwork(1).Data(4);
    udcb = block.Dwork(1).Data(5);
    block.OutputPort(1).Data = single([ ialpha ; sqrt(3)/2*ibeta + 0.5* ialpha ; -sqrt(3)/2*ibeta - 0.5* ialpha ]);
    block.OutputPort(2).Data = single([ udcb ]);
    block.OutputPort(3).Data = single([ theta*360/2/3.14 ]);
    global F3MC_omega;
    F3MC_omega = omega;
    %block.OutputPort(4).Data = single([ 5.5223e+04/omega ]);
    

%     block.OutputPort(1).Data = single([ 0 ; 0 ; 0 ]);
%     block.OutputPort(2).Data = single([ 12 ]);
%     block.OutputPort(3).Data = single([ 0 ]);
%     block.OutputPort(4).Data = single([ 3000*pi/30 ]);
    
%endfunction

function Update(block)
    %parametry
    Ts = block.DialogPrm(1).Data(1);
    Ke = single(0.0249);
    L = single(4e-4);
    R = single(0.4);
    J = single(8e-5);
    pp = single(3);
    
    global F3MC_alfa;
    global F3MC_beta;
    ualpha = F3MC_alfa * block.Dwork(1).Data(5);
    ubeta = F3MC_beta * block.Dwork(1).Data(5);
    
    ialpha = block.Dwork(1).Data(1);
    ibeta = block.Dwork(1).Data(2);
    theta = block.Dwork(1).Data(3);
    omega = block.Dwork(1).Data(4);
    
    % rovnice
    x(1) = ialpha - R*Ts/L*ialpha + Ts*Ke*omega*sin(theta) + Ts/L*ualpha;
    x(2) = ibeta - R*Ts/L*ibeta - Ts*Ke*omega*cos(theta) + Ts/L*ubeta;
    x(3) = omega + 3/2*Ts*pp*pp*Ke/J*ibeta*cos(theta) - 3/2*Ts*pp*pp*Ke/J*ialpha*sin(theta);
    x(4) = theta + Ts*omega;
    x(5) = block.Dwork(1).Data(5);
    
    block.Dwork(1).Data = x;
    % no state
%endfunction

