function close_fomo_visualization()
% Close only the optional FOMO figures belonging to this project copy.
root=fileparts(mfilename('fullpath')); setup_project(false);
project=ei_hash(root);
figures=findall(groot,'Type','figure','-regexp','Tag',['^EI_FOMO_' project(1:12) '_Frame_']);
if ~isempty(figures),close(figures);end
end
