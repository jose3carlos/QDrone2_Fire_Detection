function digest = ei_hash(value, isFile)
% SHA-256, without external executables or toolboxes.
if nargin < 2, isFile = false; end
md = java.security.MessageDigest.getInstance('SHA-256');
if isFile
    f = fopen(value,'rb'); assert(f>=0,'EI:Read','Cannot read %s',value);
    cleaner = onCleanup(@() fclose(f)); %#ok<NASGU>
    while true
        data = fread(f,1048576,'*uint8');
        if isempty(data), break; end
        md.update(typecast(data,'int8'));
    end
else
    md.update(typecast(unicode2native(char(value),'UTF-8'),'int8'));
end
digest = lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
