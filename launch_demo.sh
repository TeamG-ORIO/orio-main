#!/bin/bash
# One-command ORIO demo bring-up via tmuxifier.
#   bash launch_demo.sh              # full demo
#   bash launch_demo.sh --no-vacuum  # dry-run, no pneumatics
#   bash launch_demo.sh --dexnet     # add the DexNet suction-planner pane
#   bash launch_demo.sh --no-logging # skip the rerun recorder + event logging
#   bash launch_demo.sh --live       # recorder also streams to a running `rerun` viewer
# Env: ORIO_FRANKAPY, ORIO_PERCEPTION_ASSETS, ORIO_CONTAINER, ORIO_RUN_DIR.
# No `set -u`: tmuxifier init.sh is not nounset-safe.
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORIO_REPO="$REPO"
export ORIO_BRINGUP_TMUX="$REPO/src/devel_packages/orio_bringup/tmux"
export TMUXIFIER_LAYOUT_PATH="$ORIO_BRINGUP_TMUX/layouts"
# One run folder per session (docs/LOGGING.md).
export ORIO_RUN_ID="${ORIO_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export ORIO_RUN_DIR="${ORIO_RUN_DIR:-$REPO/logging/rerun/$ORIO_RUN_ID}"
export ORIO_LOGGING="${ORIO_LOGGING:-1}"

for arg in "$@"; do
    case "$arg" in
        --no-vacuum|--disable-pneumatics) export ORIO_NO_VACUUM=1 ;;
        --dexnet) export ORIO_DEXNET=1 ;;
        --no-logging) export ORIO_LOGGING=0 ;;
        --live) export ORIO_RERUN_LIVE=1 ;;
        -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

# Locate tmuxifier.
if ! command -v tmuxifier >/dev/null 2>&1; then
    if [ -x "$HOME/.tmuxifier/bin/tmuxifier" ]; then
        export PATH="$HOME/.tmuxifier/bin:$PATH"
    else
        echo "ERROR: tmuxifier not found." >&2
        echo "  Install (bash + tmux only, no root):" >&2
        echo "    git clone https://github.com/jimeh/tmuxifier.git ~/.tmuxifier" >&2
        echo "  Then re-run this script." >&2
        exit 1
    fi
fi

eval "$(tmuxifier init -)"
exec tmuxifier load-session orio
