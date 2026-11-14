#!/usr/bin/env bash
# run_scan_demo.sh - ONE command on the PC for the autonomous RFID scan demo.
#
#   demo/run_scan_demo.sh [--conf FILE] [--local] [--no-gui] [--stop]
#
# It starts, in this order, whatever is not already running:
#   1. the inventory receiver on this PC          sync/pc_receiver.py   (builds a test database if DB does not exist)
#   2. the robot's agent, over ssh                orio_stretch/scripts/demo_agent.py
#   3. the item localiser, when ENGINE=pc         loc/live/engine.py    (skipped with a note if it is not there)
#   4. the window                                 demo/scan_dashboard.py
# When the window is closed it stops what it started; the robot's agent is left alone while a run is still going.
#
#   --conf FILE   settings (default demo/demo.conf): the two addresses, the database, the site, the method
#   --local       everything on this machine, no ssh: for a rehearsal on the robot itself (replay an old run)
#   --no-gui      start 1-3 and wait (Ctrl-C stops them): for a PC without a screen, or to look at the logs
#   --stop        stop everything this script starts, including the robot's agent, and exit
#
# Needs: Python 3.8+ with tkinter on the PC; an ssh key from the PC to the robot (ssh-copy-id); both machines on
# one network (Tailscale today, the team router on demo day: change ROBOT_HOST and PC_HOST in the conf file).
# Logs: ~/orio_scan/logs on the PC, ~/orio/maps/logs/demo_agent.log on the robot.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PKG="$(cd "$HERE/.." && pwd)"
CONF="$HERE/demo.conf"; LOCAL=false; GUI=true; STOP=false
while [ $# -gt 0 ]; do
  case "$1" in
    --conf) CONF="$2"; shift;; --local) LOCAL=true;; --no-gui) GUI=false;; --stop) STOP=true;;
    -h|--help) sed -n '2,20p' "$0"; exit 0;;
    *) echo "unknown option $1" >&2; exit 1;;
  esac; shift
done
say() { echo "[scan demo] $*"; }
die() { echo "[scan demo] ERROR: $*" >&2; exit 1; }
[ -f "$CONF" ] || die "no conf file: $CONF (copy demo.conf.example to demo.conf and fill in the two addresses)"
while IFS='=' read -r k v; do
  case "$k" in ''|\#*) continue;; esac
  printf -v "$k" '%s' "$v"
done < <(grep -E '^[A-Z_]+=' "$CONF")
: "${ROBOT_HOST:?}" "${ROBOT_USER:?}" "${PC_HOST:?}" "${AGENT_PORT:=8123}" "${RECEIVER_PORT:=8000}" "${PYTHON:=python3}"
: "${ENGINE:=pc}" "${METHOD:=F4}" "${CELLS:=all}"
[ -n "${ROBOT_REPO:-}" ] || ROBOT_REPO='~/orio-main'      # stays unexpanded: it is a path on the robot
[ -n "${DB:-}" ] || DB='~/orio_scan/master.db'
DB="${DB/#\~/$HOME}"; WORK="$(dirname "$DB")"; LOGS="$WORK/logs"; mkdir -p "$LOGS"
if $LOCAL; then ROBOT_HOST=127.0.0.1; PC_HOST=127.0.0.1; fi
AGENT="http://$ROBOT_HOST:$AGENT_PORT"; RECV="http://127.0.0.1:$RECEIVER_PORT"; PC_URL="http://$PC_HOST:$RECEIVER_PORT"
# one shared secret between the window and the robot's agent, made once and kept on this PC
TOKEN_FILE="$HOME/.orio_demo_token"
[ -s "$TOKEN_FILE" ] || { (umask 077; head -c 16 /dev/urandom | od -An -tx1 | tr -d ' \n' > "$TOKEN_FILE"); }
TOKEN="$(cat "$TOKEN_FILE")"
on_robot() { if $LOCAL; then bash -c "$1"; else ssh -o BatchMode=yes -o ConnectTimeout=8 "$ROBOT_USER@$ROBOT_HOST" "$1"; fi; }
agent_get() { curl -s -m 4 -H "X-Orio-Token: $TOKEN" "$AGENT/status"; }
field() { "$PYTHON" -c "import json,sys; d=json.load(sys.stdin); v=d$1; print('' if v is None else v)" 2>/dev/null; }
# the agent writes its pid next to its log: stopping by pid cannot hit an editor or shell that merely names the script
RLOGS='${ORIO_DATA:-$HOME/orio}/maps/logs'      # expanded on the robot
stop_agent() { on_robot "f=$RLOGS/demo_agent.pid; [ -s \$f ] && kill -TERM \$(cat \$f) 2>/dev/null; true"; }
R_PID=""; E_PID=""
# The localiser runs inside a small loop that starts it again if it stops by itself (engine_loop.pid = the loop,
# engine.pid = the localiser). Stopping means: the loop first, then the localiser, then WAIT until it is gone. A
# new one started while the old one was still finishing ran abuela's GPU out of memory (2026-11-13).
stop_engine() {
  local lp ep; lp="$(cat "$LOGS/engine_loop.pid" 2>/dev/null)"; ep="$(cat "$LOGS/engine.pid" 2>/dev/null)"
  [ -n "$lp" ] && kill -TERM "$lp" 2>/dev/null
  if [ -n "$ep" ] && kill -TERM "$ep" 2>/dev/null; then
    for _ in $(seq 1 60); do kill -0 "$ep" 2>/dev/null || break; sleep 0.5; done
    kill -KILL "$ep" 2>/dev/null
  fi
  rm -f "$LOGS/engine_loop.pid" "$LOGS/engine.pid"
}
# TERM, not INT: bash starts background jobs of a script with SIGINT ignored
stop_pc() { [ -n "$E_PID" ] && stop_engine; [ -n "$R_PID" ] && kill -TERM "$R_PID" 2>/dev/null; rm -f "$LOGS/receiver.pid"; true; }

if $STOP; then
  say "stopping the robot's agent, the localiser and the receiver ..."
  stop_agent
  stop_engine
  [ -s "$LOGS/receiver.pid" ] && kill -TERM "$(cat "$LOGS/receiver.pid")" 2>/dev/null; rm -f "$LOGS/receiver.pid"
  exit 0
fi

"$PYTHON" -c 'import tkinter' 2>/dev/null || { $GUI && die "$PYTHON has no tkinter (sudo apt install python3-tk), or set PYTHON in $CONF"; }

# ---- 1. receiver --------------------------------------------------------------------------------------------
if [ ! -f "$DB" ]; then
  say "no database at $DB: building a test one (seed inventory + the item tags of RFID/tag_registry.csv)"
  "$PYTHON" "$PKG/sync/make_dummy_db.py" "$DB" --registry "$PKG/RFID/tag_registry.csv" || die "could not build $DB"
fi
if curl -s -m 2 "$RECV/health" | grep -q '"ok": true'; then
  say "receiver already running on port $RECEIVER_PORT (left as it is)"
else
  "$PYTHON" -u "$PKG/sync/pc_receiver.py" --db "$DB" --port "$RECEIVER_PORT" >> "$LOGS/pc_receiver.log" 2>&1 &
  R_PID=$!; echo "$R_PID" > "$LOGS/receiver.pid"; sleep 1
  kill -0 "$R_PID" 2>/dev/null || die "the receiver did not start: $(tail -2 "$LOGS/pc_receiver.log")"
  say "receiver started (database $DB, log $LOGS/pc_receiver.log)"
fi
trap 'stop_pc' EXIT

# ---- 2. the robot's agent -----------------------------------------------------------------------------------
ST="$(agent_get)"
if [ -n "$ST" ] && [ "$(echo "$ST" | field "['run']['alive']")" != "True" ] && [ "$(echo "$ST" | field "['stale']")" = "True" ]; then
  say "the robot's agent is older than its files on the robot: starting it again"; ST=""
fi
if [ -n "$ST" ] && [ "$(echo "$ST" | field "['pc']")" = "$PC_URL" ] && [ "$(echo "$ST" | field "['engine']")" = "$ENGINE" ]; then
  say "robot agent already running on $ROBOT_HOST (left as it is)"
else
  if [ -n "$ST" ] && [ "$(echo "$ST" | field "['run']['alive']")" = "True" ]; then
    die "the robot's agent is running with other settings AND a run is going: stop the run first (or run with the same conf)"
  fi
  say "starting the robot's agent on $ROBOT_HOST ..."
  stop_agent; sleep 1
  on_robot "mkdir -p $RLOGS; cd $ROBOT_REPO/src/devel_packages/orio_stretch || exit 1; \
            ORIO_DEMO_TOKEN=$TOKEN nohup python3 scripts/demo_agent.py --port $AGENT_PORT --pc $PC_URL --engine $ENGINE --method $METHOD \
            >> $RLOGS/demo_agent.log 2>&1 < /dev/null & sleep 0.5" \
    || die "could not start the agent over ssh ($ROBOT_USER@$ROBOT_HOST). Is the robot on? Does 'ssh $ROBOT_USER@$ROBOT_HOST' work without a password?"
  for _ in $(seq 1 20); do ST="$(agent_get)"; [ -n "$ST" ] && break; sleep 0.5; done
  [ -n "$ST" ] || die "the agent does not answer on $AGENT (on the robot: tail ~/orio/maps/logs/demo_agent.log; firewall on port $AGENT_PORT?)"
  say "robot agent up ($(echo "$ST" | field "['host']"))"
fi

# ---- 3. item localiser on this PC ---------------------------------------------------------------------------
if [ "$ENGINE" = pc ]; then
  if [ -f "$PKG/loc/live/engine.py" ]; then
    stop_engine                    # one left over from an earlier session must be gone before a new one starts
    ( while true; do
        "$PYTHON" -u "$PKG/loc/live/engine.py" --runs-dir "$WORK/runs" --post "$RECV" --cells "$CELLS" &
        echo $! > "$LOGS/engine.pid"; wait $!
        rc=$?; echo "$(date +%T)  the localiser stopped by itself (exit $rc): starting it again in 5 s"
        curl -s -m 2 -H 'Content-Type: application/json' -d "{\"note\": \"stopped by itself (exit $rc), starting again: see logs/engine.log\", \"error\": true}" "$RECV/engine" >/dev/null
        sleep 5
      done ) >> "$LOGS/engine.log" 2>&1 &
    E_PID=$!; echo "$E_PID" > "$LOGS/engine_loop.pid"; say "item localiser started (log $LOGS/engine.log)"
  else
    say "NOTE: loc/live/engine.py is not there yet: presence works, items get no location"
  fi
fi

# ---- 4. the window ------------------------------------------------------------------------------------------
if $GUI; then
  ORIO_DEMO_TOKEN="$TOKEN" "$PYTHON" "$HERE/scan_dashboard.py" --conf "$CONF" --agent "$AGENT" --receiver "$RECV"
else
  say "running without the window. Ctrl-C stops the receiver and the localiser."
  trap 'exit 0' INT; while true; do sleep 1; done
fi
if [ "$(agent_get | field "['run']['alive']")" = "True" ]; then
  say "a run is still going: the robot's agent is left running (stop the run from a new window, or: $0 --stop)"
else
  stop_agent; say "robot agent stopped"
fi
