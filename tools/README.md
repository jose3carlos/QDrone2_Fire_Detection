# Local build tools

For local validation, a portable MinGW-w64 GCC 8.1.0 x64 POSIX/SEH toolchain was downloaded from the upstream mingw-w64 distribution:

https://downloads.sourceforge.net/project/mingw-w64/Toolchains%20targetting%20Win64/Personal%20Builds/mingw-builds/8.1.0/threads-posix/seh/x86_64-8.1.0-release-posix-seh-rt_v6-rev0.7z

It is extracted under `mingw64/`; upstream licenses are retained. No global installer is required. `setup_project` uses it only for the current MATLAB process. This generated tool folder and archive are excluded from version control. A fresh checkout may instead use a compiler installed via MATLAB Add-Ons or Visual Studio Build Tools; see the installation guide.
