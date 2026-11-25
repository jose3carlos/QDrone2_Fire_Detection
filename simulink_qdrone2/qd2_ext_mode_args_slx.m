function [args, config] = qd2_ext_mode_args_slx(archivo)
%QD2_EXT_MODE_ARGS_SLX ExtModeMexArgs GUARDADO en un .slx (configuración activa), sin cargarlo.
%   [args, config] = qd2_ext_mode_args_slx(archivo) lee el valor tal como está en el archivo
%   (simulink/configSetInfo.xml -> configSetN.xml activo) y devuelve el texto ya sin escapes
%   XML (&apos; -> ') y el nombre de la configuración activa.
%
%   Por qué leer el archivo y no get_param después de cargar: QUARC evalúa ExtModeMexArgs al
%   ABRIR el modelo y al empezar el build, ANTES del InitFcn (hallazgo de la PC del laboratorio,
%   docs/HALLAZGOS_PC_LAB_23NOV.md fila 1). Si el .slx guardado lo trae sin comillas simples,
%   QUARC falla con "This statement is incomplete." y no aparece la pestaña HARDWARE, aunque el
%   InitFcn (qd2_cargar_config(bdroot)) lo corrija después. Lo usan construir_modelos (comprueba
%   lo que guarda y exporta) y test_qd2_lib/test_ext_mode_args_slx.

tmp = tempname;
mkdir(tmp);
limpiar = onCleanup(@() rmdir(tmp, 's'));
unzip(archivo, tmp);
info = fileread(fullfile(tmp, 'simulink', 'configSetInfo.xml'));
t = regexp(info, '<ConfigSet PartName="/([^"]+)" Active="true">([^<]*)</ConfigSet>', 'tokens', 'once');
if isempty(t)
    error('qd2_ext_mode_args_slx:sin_activa', 'No encuentro la configuración activa en %s', archivo);
end
config = t{2};
xml = fileread(fullfile(tmp, strrep(t{1}, '/', filesep)));
v = regexp(xml, '<P Name="ExtModeMexArgs"[^>/]*>([^<]*)</P>', 'tokens', 'once');
if ~isempty(v)
    args = v{1};
elseif ~isempty(regexp(xml, '<P Name="ExtModeMexArgs"[^>/]*/>', 'once'))
    args = '';                                       % campo vacío (p. ej. la librería)
else
    error('qd2_ext_mode_args_slx:sin_valor', 'No encuentro ExtModeMexArgs en %s (%s)', archivo, t{1});
end
args = strrep(strrep(strrep(strrep(strrep(args, '&apos;', ''''), '&quot;', '"'), ...
    '&lt;', '<'), '&gt;', '>'), '&amp;', '&');
end
