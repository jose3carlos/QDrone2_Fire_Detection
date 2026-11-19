function sequence = prepare_image_sequence(images,times,info)
% Preload a synchronized image/event pair. Times are SIMULATION seconds.
cfg=config();
if nargin<3
    data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact'); info=data.artifact.metadata;
end
assert(iscell(images) && numel(images)==numel(times) && ~isempty(images), ...
    'EI:Sequence','Supply an image cell array and one timestamp per image.');
times=double(times(:)); dt=cfg.image.sampleTime;
assert(all(isfinite(times)) && all(times>=0) && all(diff(times)>=2*dt-1e-9), ...
    'EI:Events','Events must be ordered and separated by at least two source sample times.');
assert(all(abs(times/dt-round(times/dt))<1e-7),'EI:TimeGrid', ...
    'Event times must lie on the configured simulation sample grid (%.9g s).',dt);
t=unique([0;times;times+dt;times(end)+2*dt]);
if info.channels==1
    frames=zeros(double(info.height),double(info.width),numel(t),'uint8');
else
    frames=zeros(double(info.height),double(info.width),3,numel(t),'uint8');
end
trigger=false(numel(t),1); frame=zeros(double(info.height),double(info.width),info.channels,'uint8');
for k=1:numel(t)
    event=find(abs(times-t(k))<dt*1e-6,1);
    if ~isempty(event), frame=prepare_image(images{event},info); trigger(k)=true; end
    if info.channels==1,frames(:,:,k)=frame;else,frames(:,:,:,k)=frame;end
end
sequence.image=timeseries(frames,t,'IsTimeFirst',false);
sequence.newImage=timeseries(trigger,t);
sequence.image=setinterpmethod(sequence.image,'zoh');
sequence.newImage=setinterpmethod(sequence.newImage,'zoh');
sequence.stopTime=t(end); sequence.eventTimes=times;
end
