#!/bin/bash
# Lanza qdrone2_camera_streamer.py en el Jetson del dron, lo relanza si muere y sobrevive al cierre de ssh.
#
# Copiar al dron (junto al streamer, en la misma carpeta):
#   scp web/drone/qdrone2_camera_streamer.py web/drone/correr_streamer.sh nvidia@<IP_DRON>:~/
# Uso (en el dron o desde la laptop por ssh; la contraseña la teclea el usuario):
#   ssh nvidia@<IP_DRON> "bash ~/correr_streamer.sh --camara auto --serie <serie>"   arranca en segundo plano
#   ssh nvidia@<IP_DRON> "bash ~/correr_streamer.sh --estado"                         ¿corre? + últimas líneas del log
#   ssh nvidia@<IP_DRON> "bash ~/correr_streamer.sh --detener"                        detiene el bucle y el streamer
#   ssh nvidia@<IP_DRON> "tail -f ~/streamer_camara.log"                              ver el log en vivo
#   bash ~/correr_streamer.sh --primer-plano --camara auto                            el bucle en esta terminal (Ctrl+C)
# Los argumentos (salvo --detener/--estado/--primer-plano) pasan tal cual al streamer.
#
# Qué hace: un bucle (setsid + nohup: no depende de la sesión ssh) que corre el streamer con python3 y, si termina
# (error, código 3 del vigía de la cámara, 4 si la cámara no vuelve, kill), lo relanza tras una espera creciente
# (2, 4, 8 ... 30 s; vuelve a 2 s si el streamer duró más de 60 s). Log: ~/streamer_camara.log (se rota a .1 al
# pasar de 5 MB al arrancar). PID del bucle: ~/.streamer_camara.pid (solo se cree si /proc dice que es este script).
# No instala servicios del sistema ni pide contraseñas.
# Variables opcionales: STREAMER_LOG, STREAMER_PID, PYTHON (por defecto python3).

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STREAMER="$DIR/qdrone2_camera_streamer.py"
LOG="${STREAMER_LOG:-$HOME/streamer_camara.log}"
PIDF="${STREAMER_PID:-$HOME/.streamer_camara.pid}"
PY="${PYTHON:-python3}"

hora() { date '+%Y-%m-%d %H:%M:%S'; }

pid_bucle() {
    # PID del bucle si está vivo Y es este script; vacío si no. El archivo sobrevive a los reinicios del Jetson
    # (cambio de batería sin --detener): el PID viejo puede ser ya otro proceso, que no se toca.
    [ -f "$PIDF" ] || return 0
    local p
    p="$(cat "$PIDF" 2>/dev/null)"
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
        if [ -r "/proc/$p/cmdline" ] && ! tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null \
                | grep -qF "$(basename "$0")"; then
            return 0
        fi
        echo "$p"
    fi
}

bucle() {
    echo $$ > "$PIDF"
    local hijo="" detener=0 espera=2 rc t0
    trap 'detener=1; [ -n "$hijo" ] && kill -TERM "$hijo" 2>/dev/null' TERM INT
    trap '' HUP
    echo "[$(hora)] correr_streamer: bucle PID $$, streamer $STREAMER $*"
    while [ "$detener" -eq 0 ]; do
        t0=$(date +%s)
        "$PY" -u "$STREAMER" "$@" &
        hijo=$!
        wait "$hijo"
        rc=$?
        # wait regresa antes si llega una señal atrapada: esperar a que el streamer termine de verdad
        while kill -0 "$hijo" 2>/dev/null; do
            wait "$hijo"
            rc=$?
        done
        hijo=""
        [ "$detener" -eq 0 ] || break
        if [ $(( $(date +%s) - t0 )) -gt 60 ]; then
            espera=2
        fi
        echo "[$(hora)] correr_streamer: el streamer terminó (código $rc); se relanza en $espera s"
        sleep "$espera" &
        wait $!
        espera=$(( espera * 2 ))
        [ "$espera" -le 30 ] || espera=30
    done
    echo "[$(hora)] correr_streamer: detenido"
    rm -f "$PIDF"
}

case "$1" in
    --bucle)
        shift
        bucle "$@"
        ;;
    --primer-plano)
        shift
        p="$(pid_bucle)"
        if [ -n "$p" ]; then
            echo "Ya corre un streamer en segundo plano (PID $p): bash $0 --detener"
            exit 1
        fi
        bucle "$@"
        ;;
    --detener)
        p="$(pid_bucle)"
        if [ -z "$p" ]; then
            echo "No hay streamer corriendo con correr_streamer.sh."
            rm -f "$PIDF"
            exit 0
        fi
        # el bucle es líder de su grupo (setsid): TERM al grupo = bucle + streamer
        kill -TERM -- "-$p" 2>/dev/null || kill -TERM "$p"
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$p" 2>/dev/null || break
            sleep 0.5
        done
        if kill -0 "$p" 2>/dev/null; then
            echo "No terminó en 5 s: KILL"
            kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p"
        fi
        rm -f "$PIDF"
        echo "Streamer detenido."
        ;;
    --estado)
        p="$(pid_bucle)"
        if [ -n "$p" ]; then
            echo "Streamer corriendo (bucle PID $p). Log: $LOG"
        else
            echo "Streamer NO corre. Log: $LOG"
        fi
        [ -f "$LOG" ] && tail -n 15 "$LOG"
        ;;
    *)
        p="$(pid_bucle)"
        if [ -n "$p" ]; then
            echo "Ya corre (bucle PID $p). Para relanzar con otros argumentos: bash $0 --detener y vuelve a lanzarlo."
            exit 0
        fi
        if [ ! -f "$STREAMER" ]; then
            echo "No encuentro $STREAMER (copia qdrone2_camera_streamer.py a la misma carpeta que este script)."
            exit 1
        fi
        if [ -f "$LOG" ] && [ "$(wc -c < "$LOG")" -gt 5000000 ]; then
            mv -f "$LOG" "$LOG.1"
        fi
        if command -v setsid >/dev/null 2>&1; then
            setsid nohup bash "$0" --bucle "$@" >> "$LOG" 2>&1 < /dev/null &
        else
            nohup bash "$0" --bucle "$@" >> "$LOG" 2>&1 < /dev/null &
        fi
        sleep 1
        p="$(pid_bucle)"
        if [ -n "$p" ]; then
            echo "Streamer lanzado en segundo plano (bucle PID $p). Log: $LOG"
            echo "Detener: bash $0 --detener · Estado: bash $0 --estado"
        else
            echo "No arrancó; revisa $LOG:"
            [ -f "$LOG" ] && tail -n 15 "$LOG"
            exit 1
        fi
        ;;
esac
