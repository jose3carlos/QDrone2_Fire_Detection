function ei_assert_stopped()
if ~license('test','Simulink'), return; end
models=find_system('SearchDepth',0,'Type','block_diagram');
for i=1:numel(models)
    if strcmp(get_param(models{i},'BlockDiagramType'),'model')
        assert(strcmp(get_param(models{i},'SimulationStatus'),'stopped'),'EI:SimulationActive', ...
            'Stop simulation before rebuilding, switching or opening standalone inference.');
    end
end
end
