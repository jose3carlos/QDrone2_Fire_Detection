function figureHandle=ei_show_fomo_frame(image,result,index,total,batchID)
% One reusable figure per image index. Grayscale pixels are never modified.
% Base MATLAB graphics only: no Image Processing/Computer Vision Toolbox.
assert(isa(image,'uint8') && isequal(size(image),[180 180]),'EI:VisualizationImage','Invalid display image.');
root=fileparts(mfilename('fullpath')); project=ei_hash(root); project=project(1:12);
tag=sprintf('EI_FOMO_%s_Frame_%u',project,index);
figures=findall(groot,'Type','figure','Tag',tag);
if isempty(figures)
    figureHandle=figure('Name',sprintf('FOMO image %u / %u',index,total), ...
        'NumberTitle','off','Tag',tag,'Color','w');
else
    figureHandle=figures(1);
end
% Updating the same windows bounds resource use across arbitrarily many batches.
set(figureHandle,'Visible','on','Name',sprintf('FOMO image %u / %u | batch %s',index,total,num2str(batchID)));
ax=findall(figureHandle,'Type','axes','Tag','EI_FOMO_Axes');
if isempty(ax),ax=axes('Parent',figureHandle,'Tag','EI_FOMO_Axes');else,ax=ax(1);cla(ax);end
imageHandle=image_object(ax,image); %#ok<NASGU>
colormap(ax,gray(256)); axis(ax,'image');
set(ax,'CLim',[0 255],'XLim',[0.5 180.5],'YLim',[0.5 180.5],'YDir','reverse');
if result.detected
    box=double(result.boundingBox);
    assert(numel(box)==4 && all(isfinite(box)) && all(box(1:2)>=0) && all(box(3:4)>0) && ...
        box(1)+box(3)<=180 && box(2)+box(4)<=180, ...
        'EI:VisualizationBox','Bounding box is outside the displayed image.');
    % Native pixel 0 is MATLAB pixel center 1; its left/top edge is 0.5.
    rectangle(ax,'Position',[box(1:2)+0.5 box(3:4)],'EdgeColor','r','LineWidth',1.5, ...
        'Tag','EI_FOMO_Box');
    description=sprintf('%s | %.2f%%',result.label,result.confidence);
else
    description='No detection';
end
title(ax,sprintf('Image %u/%u | %s | result %s',index,total,description,num2str(result.resultID)), ...
    'Interpreter','none');
xlabel(ax,'Column'); ylabel(ax,'Row');
set(figureHandle,'UserData',struct('batchID',batchID,'imageIndex',index,'imageCount',total, ...
    'result',result));
drawnow nocallbacks; % Paint during simulation without re-entering user callbacks.
end

function handle=image_object(ax,pixels)
% Separate name because the caller's image variable shadows MATLAB image().
handle=image(ax,'CData',pixels,'CDataMapping','scaled','Tag','EI_FOMO_Pixels');
end
