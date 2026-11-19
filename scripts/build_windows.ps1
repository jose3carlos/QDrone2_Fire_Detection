param([string]$Matlab = 'matlab', [switch]$Validate)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$matlabRootLiteral = $projectRoot.Replace("'", "''").Replace('\', '/')
$task = if ($Validate) { 'run_validation' } else { 'setup_project' }
& $Matlab -batch "cd('$matlabRootLiteral'); $task;"
if ($LASTEXITCODE -ne 0) { throw "MATLAB build/validation failed with exit code $LASTEXITCODE" }
