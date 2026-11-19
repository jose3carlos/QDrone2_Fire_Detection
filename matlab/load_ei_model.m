function info = load_ei_model()
ei_assert_stopped(); close_ei_model(); cfg=config();
artifact=build_model();
addpath(artifact.gatewayPath);
setenv('PATH',[fileparts(artifact.dll) pathsep getenv('PATH')]);
info=feval(artifact.gateway,'initialize',artifact.dll,cfg.minConfidence,artifact.falseIndex, ...
    cfg.maxDetections,double(cfg.debugLog),cfg.timeoutMs);
info.booleanLabels=artifact.booleanLabels;
info.resizeMode=artifact.metadata.resizeMode;
ei_session('set',struct('artifact',artifact,'info',info));
if cfg.minConfidence<info.sdkThreshold
    warning('EI:SDKThreshold','SDK already discards detections below %.3f; lower application thresholds cannot recover them.',info.sdkThreshold);
end
end
