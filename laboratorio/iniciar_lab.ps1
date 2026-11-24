# Lanzador de la PC del laboratorio (Windows PowerShell 5.1). Lo abren INICIAR_EXPLORADOR.bat,
# INICIAR_DUAL.bat y DETENER_DRON.bat. Hace, en orden y preguntando antes de lo que mueve hardware:
#   1) revisiones (Motive, otros modelos QUARC, QUARC Target Manager, puertos)
#   2) modelo de tierra: abre MATLAB R2022a y corre laboratorio\iniciar_tierra_lab.m (build + Monitor & Tune)
#   3) estacion web (web\iniciar_web.bat --fuente simulink) -> abre la UI en el navegador
#   4) quarc_run del dron (binario PID original del Equipo2)
#   5) streamer de la camara por ssh (la contrasena la teclea el usuario)
# Nunca prueba el puerto 17000 del dron con una conexion TCP: el Target Manager del dron cierra su
# servicio si alguien conecta y cierra sin hablar QUARC (paso el 23-nov). Al dron solo se le hace ping.
param(
    [ValidateSet('solo', 'dual')][string]$Modo = 'solo',
    [switch]$Detener,     # solo detiene el stack del dron (quarc_run -q)
    [switch]$Revisar      # solo revisa y muestra lo que haria; no lanza nada
)
$ErrorActionPreference = 'Stop'
$lab = $PSScriptRoot
$raiz = Split-Path $lab -Parent
$cfgPath = if ($Modo -eq 'dual') { Join-Path $raiz 'config\estacion_dual.json' } else { Join-Path $raiz 'config\estacion.json' }
$cfg = Get-Content $cfgPath -Raw -Encoding UTF8 | ConvertFrom-Json
$matlab = 'C:\Program Files\MATLAB\R2022a\bin\matlab.exe'
$quarcDir = [Environment]::GetEnvironmentVariable('QUARC_DIR', 'Machine')
if (-not $quarcDir) { $quarcDir = 'C:\Program Files\Quanser\QUARC\' }
$quarcRun = Join-Path $quarcDir 'quarc_run.exe'
$binDir = Join-Path $raiz 'simulink_qdrone2\binarios_dron'
$binDron = 'QD2_DroneStack_PID_2021a.rt-linux_qdrone2'
$ipPc = if ($cfg.red.ip_pc) { $cfg.red.ip_pc } else { '192.168.2.11' }
$puertoWeb = if ($cfg.red.puerto_web) { [int]$cfg.red.puerto_web } else { 8080 }

# drones a lanzar: solo -> mision.dron_solo (1 por defecto); dual -> 1 y 2
$drones = @()
if ($Modo -eq 'dual') { $ids = 1, 2 } else { $ids = @($(if ($cfg.mision.dron_solo) { [int]$cfg.mision.dron_solo } else { 1 })) }
foreach ($i in $ids) {
    $ip = $cfg.red."ip_dron$i"
    $pt = $cfg.red."puerto_dron$i"
    $drones += [pscustomobject]@{ Id = $i; Ip = $ip; Puerto = $(if ($pt) { [int]$pt } else { 18000 + $i }) }
}

function Escribir-Titulo($t) { Write-Host ''; Write-Host ('=== ' + $t + ' ===') -ForegroundColor Cyan }
function Preguntar($t) { if ($Revisar) { Write-Host ('[revisar] ' + $t + ' -> se responderia aqui') -ForegroundColor DarkGray; return $false }; $r = Read-Host ($t + ' (S/N)'); return ($r -match '^[sS]') }
function Escuchando($puertos) {
    $l = Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Where-Object { $puertos -contains $_.LocalPort }
    return @($l | Select-Object -ExpandProperty LocalPort -Unique)
}
function Comando-Dron($d) {
    return @('-r', '-t', "tcpip://$($d.Ip):17000", $binDron, '-uri', "tcpip://$($d.Ip):17001?retries=10", '-URI_Host', "tcpip://${ipPc}:$($d.Puerto)")
}

# ------------------------------------------------------------------ detener el dron
if ($Detener) {
    Escribir-Titulo 'Detener el stack del dron'
    Write-Host 'Solo con el dron EN EL PISO. Si esta en el aire: Takeoff/Land abajo o LAND ALL primero.' -ForegroundColor Yellow
    if (-not (Preguntar 'El dron esta en el piso?')) { Write-Host 'Cancelado.'; if (-not $Revisar) { Read-Host 'Enter para cerrar' }; exit 0 }
    Push-Location $binDir
    try {
        foreach ($d in $drones) {
            if (-not $d.Ip) { continue }
            Write-Host ("quarc_run -q dron {0} ({1})" -f $d.Id, $d.Ip)
            & $quarcRun -q -t "tcpip://$($d.Ip):17000" $binDron
        }
    } finally { Pop-Location }
    Read-Host 'Listo. Enter para cerrar'
    exit 0
}

# ------------------------------------------------------------------ 1) revisiones
Escribir-Titulo ("Lanzador QDrone 2 - modo " + $Modo + " (" + (Split-Path $cfgPath -Leaf) + ")")
$tm = Get-CimInstance Win32_Service -Filter "Name='QUARC Target Manager'" -ErrorAction SilentlyContinue
if (-not $tm -or $tm.State -ne 'Running' -or -not (Escuchando @(17000))) {
    Write-Host 'El QUARC Target Manager de esta PC no escucha en 17000. En PowerShell como ADMINISTRADOR:' -ForegroundColor Red
    Write-Host '  Restart-Service -Name "QUARC Target Manager" -Force'
    if (-not $Revisar) { Read-Host 'Enter para salir'; exit 1 }
} else { Write-Host 'QUARC Target Manager: OK (17000)' -ForegroundColor Green }

while (Get-Process Motive -ErrorAction SilentlyContinue) {
    Write-Host 'Motive esta abierto: el bloque OptiTrack de tierra no puede usar las camaras. Cierralo.' -ForegroundColor Yellow
    if ($Revisar) { break }
    Read-Host 'Enter cuando Motive este cerrado'
}

$hijos = @()
if ($tm) { $hijos = @(Get-CimInstance Win32_Process | Where-Object { $_.ParentProcessId -eq $tm.ProcessId -and $_.Name -ne 'conhost.exe' }) }
$ajenos = @($hijos | Where-Object { $_.Name -notlike 'QD2_MissionCtrl_Dual_PID*' })
if ($ajenos.Count) {
    Write-Host 'OJO: hay otros modelos QUARC corriendo (pueden tener las camaras OptiTrack o los puertos):' -ForegroundColor Yellow
    $ajenos | ForEach-Object { Write-Host ("  {0} (PID {1})" -f $_.Name, $_.ProcessId) }
    Write-Host '  Detenlos (bandeja de QUARC -> Stop, o Stop-Process como administrador) antes de seguir.'
    if (-not $Revisar) { Read-Host 'Enter para seguir' }
}

# ------------------------------------------------------------------ 2) modelo de tierra
Escribir-Titulo 'Modelo de tierra'
$puertosTierra = @(18001, 18002, [int]$cfg.red.puerto_puente | Where-Object { $_ })
$ya = Escuchando $puertosTierra
if ($ya.Count -eq $puertosTierra.Count) {
    Write-Host ('Ya esta corriendo (escuchan ' + ($ya -join ', ') + '). No se toca.') -ForegroundColor Green
} else {
    $mlAbierto = @(Get-Process MATLAB -ErrorAction SilentlyContinue)
    if ($mlAbierto.Count) {
        Write-Host 'Ya hay un MATLAB abierto. Opcion A: en SU consola escribe:' -ForegroundColor Yellow
        Write-Host ("  cd('" + $lab + "'); iniciar_tierra_lab('" + $Modo + "')")
        Write-Host 'Opcion B: abrir otro MATLAB desde aqui.'
    }
    if (-not $mlAbierto.Count -or (Preguntar 'Abrir un MATLAB nuevo y arrancar tierra desde aqui?')) {
        if ($Revisar) {
            Write-Host ('[revisar] abriria: "' + $matlab + '" -r "cd(''' + $lab + '''); iniciar_tierra_lab(''' + $Modo + ''')"') -ForegroundColor DarkGray
        } else {
            Write-Host 'Abriendo MATLAB R2022a. Si sale "QUARC Tip of the Day", cierrala para que siga.'
            Start-Process -FilePath $matlab -ArgumentList @('-r', ("cd('" + $lab + "'); iniciar_tierra_lab('" + $Modo + "')"))
        }
    }
    if (-not $Revisar) {
        Write-Host ('Esperando a que escuchen ' + ($puertosTierra -join ', ') + ' (hasta 10 min)...')
        $limite = (Get-Date).AddMinutes(10)
        while ((Escuchando $puertosTierra).Count -lt $puertosTierra.Count) {
            if ((Get-Date) -gt $limite) { Write-Host 'No arranco el modelo de tierra. Revisa la consola de MATLAB.' -ForegroundColor Red; Read-Host 'Enter para salir'; exit 1 }
            Start-Sleep -Seconds 5
            Write-Host -NoNewline '.'
        }
        Write-Host ''
        Write-Host 'Modelo de tierra corriendo.' -ForegroundColor Green
    }
}

# ------------------------------------------------------------------ 3) estacion web
Escribir-Titulo 'Estacion web'
$url = "http://127.0.0.1:$puertoWeb"
if ((Escuchando @($puertoWeb)).Count) {
    Write-Host ('La estacion ya corre en ' + $url + '. Abro la UI.') -ForegroundColor Green
    try {
        $e = Invoke-RestMethod "$url/api/estado" -TimeoutSec 5
        if ($e.modo.config.modo -and $e.modo.config.modo -ne $(if ($Modo -eq 'dual') { 'dual' } else { 'solo_explorador' })) {
            Write-Host ('OJO: esa estacion esta en modo ' + $e.modo.config.modo + '. Cierrala y vuelve a correr este lanzador.') -ForegroundColor Yellow
        }
    } catch { }
    if (-not $Revisar) { Start-Process $url }
} else {
    $argsWeb = '--fuente simulink'
    if ($Modo -eq 'dual') { $argsWeb += ' --config "' + $cfgPath + '"' }
    $bat = Join-Path $raiz 'web\iniciar_web.bat'
    if ($Revisar) { Write-Host ('[revisar] abriria otra ventana: "' + $bat + '" ' + $argsWeb) -ForegroundColor DarkGray }
    else {
        Start-Process -FilePath 'cmd.exe' -ArgumentList @('/k', ('"' + $bat + '" ' + $argsWeb))
        Write-Host ('Estacion arrancando en otra ventana; la UI se abre sola en ' + $url + '.') -ForegroundColor Green
    }
}

# ------------------------------------------------------------------ 4) dron(es)
foreach ($d in $drones) {
    Escribir-Titulo ("Dron " + $d.Id)
    if (-not $d.Ip) { Write-Host ("Sin IP: pon red.ip_dron{0} en la pestana Conexiones." -f $d.Id) -ForegroundColor Yellow; continue }
    $vivo = Test-Connection $d.Ip -Count 2 -Quiet -ErrorAction SilentlyContinue
    Write-Host ("{0}: ping {1}" -f $d.Ip, $(if ($vivo) { 'OK' } else { 'SIN RESPUESTA (prendelo y espera 2-3 min)' }))
    $cmd = Comando-Dron $d
    Write-Host ('quarc_run ' + ($cmd -join ' ')) -ForegroundColor DarkGray
    Write-Host 'Antes: dron en su home EN EL PISO, switches Arm / Takeoff-Land / E-Stop ABAJO.'
    if (Preguntar ("Lanzar el stack del dron {0} ahora?" -f $d.Id)) {
        Push-Location $binDir
        try { & $quarcRun @cmd } finally { Pop-Location }
        Write-Host 'Si dice "remote peer refused" en :17000: en el dron (ssh) "sudo systemctl restart quarc_target_manager" y vuelve a correr esto.' -ForegroundColor DarkGray
    }
}

# ------------------------------------------------------------------ 5) camara
$exp = $drones[0]
if ($exp.Ip) {
    Escribir-Titulo 'Camara del explorador'
    $ssh = 'ssh nvidia@' + $exp.Ip + ' "python3 ~/qdrone2_camera_streamer.py --camara auto --puerto 18800"'
    Write-Host $ssh -ForegroundColor DarkGray
    if (Preguntar 'Abrir el streamer de la camara en otra ventana (pide la contrasena del dron)?') {
        Start-Process -FilePath 'powershell.exe' -ArgumentList @('-NoExit', '-Command', $ssh)
    }
}

Escribir-Titulo 'Emergencias'
Write-Host '- Takeoff/Land ABAJO = aterrizaje ordenado (funciona aunque caiga la estacion o la red).'
Write-Host '- UI: ABORT (regresa a home) o LAND ALL. Sin estacion o red -> PAUSED (hover) en 1 s.'
Write-Host '- E-Stop corta motores: el dron CAE. Nunca bajar Arm para aterrizar ni Stop de tierra en el aire.'
Write-Host '- auto_land = false: al terminar, el dron espera en hover hasta LAND ALL.'
Write-Host '- Al terminar, con el dron en el piso: DETENER_DRON.bat y luego Stop en Simulink.'
if (-not $Revisar) { Read-Host 'Enter para cerrar esta ventana (lo demas sigue corriendo)' }
