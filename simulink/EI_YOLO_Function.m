function [confidence, objectClass, detected, status, resultID, resultUpdated] = EI_YOLO_Function(image,newImage)
%#codegen
% Rising edges are requests. Keep the full image stable for the source sample.
% A held high signal accepts ONE request. A low sample re-arms the input.
persistent previous heldConfidence heldClass heldDetected heldStatus heldID
if isempty(previous)
    previous=false; heldConfidence=single(0); heldClass=false;
    heldDetected=false; heldStatus=int32(0); heldID=uint64(0);
end
coder.cinclude('ei_wrapper.h');
resultUpdated=false;
if newImage && ~previous
    c=single(0); o=uint8(0); d=uint8(0); id=uint64(0);
    s=int32(0);
    s=coder.ceval('ei_sim_infer',coder.rref(image),uint64(numel(image)), ...
        uint32(size(image,1)),uint32(size(image,2)),coder.wref(c), ...
        coder.wref(o),coder.wref(d),coder.wref(id));
    heldConfidence=c; heldClass=(o~=0); heldDetected=(d~=0);
    heldStatus=s; heldID=id; resultUpdated=true;
end
previous=newImage;
confidence=heldConfidence; objectClass=heldClass; detected=heldDetected;
status=heldStatus; resultID=heldID;
end
