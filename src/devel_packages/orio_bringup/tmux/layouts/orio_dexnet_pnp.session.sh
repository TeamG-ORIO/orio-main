# tmuxifier session layout for the DexNet pick-and-place test (no labelling).
#
# A stripped-down copy of orio.session.sh: single arm (robot 1 = iam-doc), DexNet
# grasp backend, drop in DROP_ZONE. Removed vs. the full demo:
#   * the luisa pane (robot 2 / label_arm control PC)
#   * the label-arm half of the pipeline
#   * state_machine.py  →  replaced by dexnet_pnp.py
# DexNet is forced on and perception runs with grasp_backend=dexnet.
#
# Launched the same way as the full demo (via launch_demo.sh, which exports the
# ORIO_* env vars), just pointing tmuxifier at this layout.

REPO="${ORIO_REPO:-$HOME/16662_RobotAutonomy}"
FRANKAPY="${ORIO_FRANKAPY:-$REPO/src/git_packages/frankapy}"   # control-PC start scripts
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
WF="${ORIO_BRINGUP_TMUX:-$REPO/src/devel_packages/orio_bringup/tmux}/wait_for.sh"
NO_VACUUM="${ORIO_NO_VACUUM:-}"

# Perception source is in orio_perception; per-machine venv + weights are
# git-ignored, found via these env roots.
PERC_DIR="$REPO/src/devel_packages/orio_perception"
PERC_SCRIPTS="$PERC_DIR/scripts"
PERC_ASSETS="${ORIO_PERCEPTION_ASSETS:-$PERC_DIR}"
PERC_VENV="${ORIO_PERCEPTION_VENV:-$PERC_ASSETS/venv}"
GDINO_DIR="${ORIO_GROUNDINGDINO_DIR:-$PERC_ASSETS/GroundingDINO}"

# Run logging (docs/LOGGING.md): one folder per session, shared by recorder + perception.
# Default OFF for this test: the rerun logging library is broken on the perception
# image's Python 3.8 (rerun_bindings uses 3.9+ `list[float]` syntax), which crashes
# perception at import. Set ORIO_LOGGING=1 explicitly only once that is fixed.
LOGGING="${ORIO_LOGGING:-0}"
RUN_DIR="${ORIO_RUN_DIR:-$REPO/logging/rerun/$(date +%Y%m%d_%H%M%S)}"
LOG_ENV="ORIO_LOGGING=$LOGGING ORIO_RUN_DIR=$RUN_DIR ORIO_RUN_ID=$(basename "$RUN_DIR") ORIO_RERUN_LIVE=${ORIO_RERUN_LIVE:-0}"

# ── Per-service commands (readiness-gated) ──────────────────────────────────
# ROS1 master runs in a container (this host is 22.04/ROS2, no host roscore). --net host
# exposes it at localhost:11311 for the other containers and the control PC.
CMD_ROSCORE="bash $REPO/src/devel_packages/orio_bringup/docker/run_roscore.sh"

CMD_RECORDER="source $WF && wait_for_roscore && source $REPO/devel/setup.bash && source $PERC_VENV/bin/activate && $LOG_ENV python3 $REPO/src/devel_packages/orio_logging/recorder.py"

# Only robot 1 (iam-doc = pick_place_arm). Robot 2 (iam-luisa = label_arm) is not
# started: this test uses a single arm.
CMD_DOC="cd $FRANKAPY && source $WF && wait_for_roscore && bash ./bash_scripts/start_control_pc.sh -u student -i iam-doc"

CMD_DOCKER="cd $REPO && source $WF && wait_for_roscore && ORIO_LOGGING=$LOGGING bash orio_run_docker.sh"

# DexNet suction planner (own container: NVIDIA TF1 image, GPU). Long-running - the TF
# graph takes ~25 s to load, so it starts once here rather than per pick.
CMD_DEXNET="source $WF && wait_for_roscore && ORIO_LOGGING=$LOGGING bash $REPO/src/devel_packages/orio_bringup/docker/run_dexnet.sh && docker logs -f orio_dexnet"

CMD_CAMERAS="source $WF && wait_for_container $CONTAINER && docker exec -it $CONTAINER bash -c 'source /home/ros_ws/devel/setup.bash && roslaunch manipulation cameras.launch'"

# Perception runs in the ROS1 perception container with the DexNet grasp backend,
# gated on the planner service so it does not start before DexNet is ready.
CMD_PERCEPTION="source $WF && wait_for_topic /camera/rgb/image_raw && wait_for_topic /zedm/zed_node/rgb/image_rect_color && wait_for_service /dexnet_grasp_planner/plan_grasp && $LOG_ENV bash $REPO/src/devel_packages/orio_bringup/docker/run_perception.sh dexnet"

if [ -n "$NO_VACUUM" ]; then
    CMD_PNEU="echo '[pneumatics] DISABLED (dry-run --no-vacuum): not starting pneumatic_control. Move the cup by hand; vacuum commands and sensor checks are skipped in dexnet_pnp.'"
    PNP_ARGS=" --no-vacuum"
else
    # The ClearCore firmware is flashed once and persists, so no manual re-flash / Enter
    # prompt is needed. Start the vacuum node directly; it fails loudly if the controller
    # is not reachable on its serial port.
    CMD_PNEU="cd $REPO/src/devel_packages/orio && ORIO_LOGGING=$LOGGING python3 pneumatic_control_recovery.py"
    PNP_ARGS=""
fi

# Pass-through flags for dexnet_pnp.py, set by launch_demo.sh.
[ -n "${ORIO_PNP_CONFIRM:-}" ]       && PNP_ARGS="$PNP_ARGS --confirm"
[ -n "${ORIO_PNP_STRAIGHT_DOWN:-}" ] && PNP_ARGS="$PNP_ARGS --straight-down"

# Pick-and-place: single-arm DexNet loop, gated on /compute_grasps, run inside the
# main docker container (like state_machine.py). Replaces the state_machine pane.
CMD_PNP="source $WF && wait_for_service /compute_grasps && docker exec -e ORIO_LOGGING=$LOGGING -it $CONTAINER bash -c 'source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && python3 dexnet_pnp.py$PNP_ARGS'"

# ── Build the session ───────────────────────────────────────────────────────
session_root "$REPO"

if initialize_session "orio_dexnet_pnp"; then
    # Panes tiled into one window; re-tile between splits so they stay splittable.
    # Pane titles are used by stop_demo.sh to flush the recorder + perception first.
    new_window "orio_dexnet_pnp"
    run_cmd "$CMD_ROSCORE"
    tmux select-pane -t "$session:$window" -T roscore
    PANES=()
    TITLES=()
    if [ "$LOGGING" != "0" ]; then
        PANES+=("$CMD_RECORDER"); TITLES+=(recorder)
    fi
    # No luisa pane: robot 2 / label_arm is not used in this test.
    PANES+=("$CMD_DOC" "$CMD_DOCKER" "$CMD_CAMERAS" "$CMD_DEXNET")
    TITLES+=(doc docker cameras dexnet)
    PANES+=("$CMD_PERCEPTION" "$CMD_PNEU" "$CMD_PNP")
    TITLES+=(perception pneumatics dexnet_pnp)
    for i in "${!PANES[@]}"; do
        split_v
        tmux select-layout -t "$session:$window" tiled
        run_cmd "${PANES[$i]}"
        tmux select-pane -t "$session:$window" -T "${TITLES[$i]}"
    done
    tmux select-layout -t "$session:$window" tiled
    select_window orio_dexnet_pnp
fi

finalize_and_go_to_session
