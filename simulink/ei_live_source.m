function ei_live_source(block)
% Supported Level-2 MATLAB S-function source for normal-mode simulation only.
block.NumDialogPrms=4; % H, W, sample time, channels
block.AllowSignalsWithMoreThan2D=true;
block.NumInputPorts=0; block.NumOutputPorts=2;
dims=[block.DialogPrm(1).Data block.DialogPrm(2).Data];
if block.DialogPrm(4).Data==3,dims=[dims 3];end
block.OutputPort(1).Dimensions=dims;
block.OutputPort(1).DatatypeID=3; % uint8
block.OutputPort(1).Complexity='Real';
block.OutputPort(2).Dimensions=1; block.OutputPort(2).DatatypeID=8; % boolean
block.OutputPort(2).Complexity='Real';
block.SampleTimes=[block.DialogPrm(3).Data 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@allocate);
block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Update',@update);
end
function allocate(block)
block.NumDworks=2;
block.Dwork(1).Name='frame';block.Dwork(1).Dimensions=prod(block.OutputPort(1).Dimensions);
block.Dwork(1).DatatypeID=3;block.Dwork(1).Complexity='Real';block.Dwork(1).UsedAsDiscState=true;
block.Dwork(2).Name='previousHigh';block.Dwork(2).Dimensions=1;
block.Dwork(2).DatatypeID=8;block.Dwork(2).Complexity='Real';block.Dwork(2).UsedAsDiscState=true;
end
function initialize(block)
block.Dwork(1).Data=zeros(block.Dwork(1).Dimensions,1,'uint8');
block.Dwork(2).Data=false;
end
function outputs(block)
% Do not dequeue in Outputs: solvers may call Outputs multiple times per hit.
block.OutputPort(1).Data=reshape(block.Dwork(1).Data,block.OutputPort(1).Dimensions);
block.OutputPort(2).Data=block.Dwork(2).Data;
end
function update(block)
% A high hit always transitions low, before another pending frame is consumed.
if block.Dwork(2).Data
    block.Dwork(2).Data=false;
else
    [image,~]=ei_live_mailbox('take');
    if ~isempty(image)
        block.Dwork(1).Data=image(:);block.Dwork(2).Data=true;
    end
end
end
