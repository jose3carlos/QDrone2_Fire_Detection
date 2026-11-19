function receipt = submit_live_image(image)
cfg=config(); data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');
image=prepare_image(image,data.artifact.metadata);
[accepted,id]=ei_live_mailbox('put',image);
receipt=struct('accepted',accepted,'requestID',id,'status',int32(0));
if ~accepted, receipt.status=int32(-6); end
end
