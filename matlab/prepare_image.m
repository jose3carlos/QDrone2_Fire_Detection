function image = prepare_image(image, info)
% Base MATLAB: file path or gray/RGB/RGBA array -> model-ready uint8.
% uint16, logical and finite floating point [0,1] are converted explicitly.
% File colormaps are decoded; transparency is composited against black.
if nargin<2, info=get_ei_model_info(); end
assert(isfield(info,'channels') && ismember(info.channels,[1 3]), ...
    'EI:Channels','Model metadata must specify 1 grayscale or 3 RGB channels.');
alpha=[];
if ischar(image) || isstring(image)
    assert((ischar(image) && isrow(image)) || (isstring(image) && isscalar(image)), ...
        'EI:InvalidImage','Supply one image filename.');
    assert(isfile(image),'EI:ImageFile','Image file does not exist: %s',image);
    [image,map,alpha]=imread(image); % first frame/page for multi-image formats
    if ~isempty(map)
        if isinteger(image) || islogical(image),indices=double(image)+1;
        else,indices=double(image);end
        assert(ismatrix(image) && all(indices(:)>=1 & indices(:)<=size(map,1) & indices(:)==floor(indices(:))), ...
            'EI:InvalidImage','Invalid indexed image.');
        image=reshape(map(indices(:),:),size(image,1),size(image,2),3);
    elseif size(image,3)==4
        error('EI:InvalidImage','Four-channel file data such as CMYK must first be converted explicitly to RGB.');
    end
end
assert((isnumeric(image) || islogical(image)) && isreal(image) && ~issparse(image) && ...
    ~isempty(image) && ndims(image)<=3 && ismember(size(image,3),[1 3 4]), ...
    'EI:InvalidImage','Expected a nonempty grayscale, RGB or RGBA image array, or an image filename.');
image=to_uint8(image);
if size(image,3)==4
    alpha=image(:,:,4);image=image(:,:,1:3);
end
if ~isempty(alpha)
    assert(ismatrix(alpha) && isequal(size(alpha),[size(image,1) size(image,2)]), ...
        'EI:InvalidImage','Alpha dimensions must match the image.');
    image=uint8(round(double(image).*(double(to_uint8(alpha))/255)));
end
H=double(info.height);W=double(info.width);h=size(image,1);w=size(image,2);
if h~=H || w~=W
    cfg=config();mode=cfg.image.resize;
    if strcmp(mode,'model'),mode=info.resizeMode;end
    switch mode
        case {'fit-shortest','EI_CLASSIFIER_RESIZE_FIT_SHORTEST'}
            scale=max(H/h,W/w);
            image=sample(image,((0:H-1)+.5)/scale+(h-H/scale)/2, ...
                ((0:W-1)+.5)/scale+(w-W/scale)/2);
        case {'squash','EI_CLASSIFIER_RESIZE_SQUASH'}
            image=sample(image,((0:H-1)+.5)*h/H,((0:W-1)+.5)*w/W);
        case {'fit-longest','EI_CLASSIFIER_RESIZE_FIT_LONGEST'}
            scale=min(H/h,W/w);hh=max(1,min(H,floor(h*scale)));ww=max(1,min(W,floor(w*scale)));
            resized=sample(image,((0:hh-1)+.5)*h/hh,((0:ww-1)+.5)*w/ww);
            image=zeros(H,W,size(image,3),'uint8');y=floor((H-hh)/2);x=floor((W-ww)/2);
            image(y+(1:hh),x+(1:ww),:)=resized;
        otherwise
            error('EI:Resize','Resize mode %s requires exact model image dimensions.',mode);
    end
end
if info.channels==1 && size(image,3)==3
    % ITU-R 601 luma, matching the exported DSP's color convention.
    image=uint8(round(.299*double(image(:,:,1))+.587*double(image(:,:,2))+.114*double(image(:,:,3))));
elseif info.channels==3 && size(image,3)==1
    image=repmat(image,1,1,3);
end
end
function image=to_uint8(image)
assert(isreal(image) && ~issparse(image),'EI:InvalidImage','Image values must be real and dense.');
if isa(image,'uint8'),return;
elseif isa(image,'uint16'),image=uint8(round(double(image)/257));
elseif islogical(image),image=uint8(image)*255;
elseif isa(image,'single') || isa(image,'double')
    assert(all(isfinite(image(:))) && all(image(:)>=0 & image(:)<=1), ...
        'EI:InvalidImage','Floating-point images must be finite and normalized to [0,1].');
    image=uint8(round(double(image)*255));
else,error('EI:InvalidImage','Supported types: uint8, uint16, logical, single/double [0,1].');
end
end
function output=sample(image,y,x)
y=max(1,min(size(image,1),floor(y)+1));x=max(1,min(size(image,2),floor(x)+1));
output=image(y,x,:);
end
