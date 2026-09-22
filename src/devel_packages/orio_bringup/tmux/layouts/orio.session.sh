# tmuxifier session layout for the ORIO demo: readiness-gated panes in one tiled
# window (roscore, recorder, 2 control pcs, docker, [dexnet], cameras, perception,
# pneumatics, state machine). Launched by launch_demo.sh (exports the ORIO_* env vars).

REPO="${ORIO_REPO:-$HOME/16662_RobotAutonomy}"
FRANKAPY="${ORIO_FRANKAPY:-$REPO/src/git_packages/frankapy}"   # control-PC start scripts
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
WF="${ORIO_BRINGUP_TMUX:-$REPO/src/devel_packages/orio_bringup/tmux}/wait_for.sh"
NO_VACUUM="${ORIO_NO_VACUUM:-}"
DEXNET="${ORIO_DEXNET:-}"                                      # set to enable the DexNet pane

# Perception source is in orio_perception; per-machine venv + weights are
# git-ignored, found via these env roots.
PERC_DIR="$REPO/src/devel_packages/orio_perception"
PERC_SCRIPTS="$PERC_DIR/scripts"
PERC_ASSETS="${ORIO_PERCEPTION_ASSETS:-$PERC_DIR}"
PERC_VENV="${ORIO_PERCEPTION_VENV:-$PERC_ASSETS/venv}"
GDINO_DIR="${ORIO_GROUNDINGDINO_DIR:-$PERC_ASSETS/GroundingDINO}"

# Run logging (docs/LOGGING.md): one folder per session, shared by recorder + perception.
LOGGING="${ORIO_LOGGING:-1}"
RUN_DIR="${ORIO_RUN_DIR:-$REPO/logging/rerun/$(date +%Y%m%d_%H%M%S)}"
LOG_ENV="ORIO_LOGGING=$LOGGING ORIO_RUN_DIR=$RUN_DIR ORIO_RUN_ID=$(basename "$RUN_DIR") ORIO_RERUN_LIVE=${ORIO_RERUN_LIVE:-0}"

# ── Per-service commands (readiness-gated) ──────────────────────────────────
# ROS1 master runs in a container (this host is 22.04/ROS2, no host roscore). --net host
# exposes it at localhost:11311 for the other containers and the control PCs.
CMD_ROSCORE="bash $REPO/src/devel_packages/orio_bringup/docker/run_roscore.sh"

CMD_RECORDER="source $WF && wait_for_roscore && source $REPO/devel/setup.bash && source $PERC_VENV/bin/activate && $LOG_ENV python3 $REPO/src/devel_packages/orio_logging/recorder.py"

CMD_DOC="cd $FRANKAPY && source $WF && wait_for_roscore && bash ./bash_scripts/start_control_pc.sh -u student -i iam-doc"
CMD_LUISA="cd $REPO && source $WF && wait_for_roscore && bash src/devel_packages/orio_bringup/start_control_pc_luisa.sh"

CMD_DOCKER="cd $REPO && source $WF && wait_for_roscore && ORIO_LOGGING=$LOGGING bash orio_run_docker.sh"

# DexNet suction planner (own container: NVIDIA TF1 image, GPU). Long-running - the TF
# graph takes ~25 s to load, so it starts once here rather than per pick.
CMD_DEXNET="source $WF && wait_for_roscore && ORIO_LOGGING=$LOGGING bash $REPO/src/devel_packages/orio_bringup/docker/run_dexnet.sh && docker logs -f orio_dexnet"

CMD_CAMERAS="source $WF && wait_for_container $CONTAINER && docker exec -it $CONTAINER bash -c 'source /home/ros_ws/devel/setup.bash && roslaunch manipulation cameras.launch'"

if [ -n "$DEXNET" ]; then
    WAIT_DEXNET="wait_for_service /dexnet_grasp_planner/plan_grasp && "
else
    WAIT_DEXNET=""
fi

# Perception runs in the ROS1 perception container (orio/perception) instead of a host
# venv: this machine is 22.04/ROS2 with no host rospy. run_perception.sh sets the ORIO_*
# asset paths and mounts the source + weights; the image carries rospy, custom_msgs, SAM,
# GroundingDINO (with a compiled _C kernel) and Open3D.
CMD_PERCEPTION="source $WF && wait_for_topic /camera/rgb/image_raw && wait_for_topic /zedm/zed_node/rgb/image_rect_color && ${WAIT_DEXNET}$LOG_ENV bash $REPO/src/devel_packages/orio_bringup/docker/run_perception.sh"

if [ -n "$NO_VACUUM" ]; then
    CMD_PNEU="echo '[pneumatics] DISABLED (dry-run --no-vacuum): not starting pneumatic_control. Move the cup by hand; vacuum commands and sensor checks are skipped in the state machine.'"
    SM_ARGS=" --no-vacuum"
else
    CMD_PNEU="echo '=====================================================' && echo ' ACTION REQUIRED: flash the vacuum firmware via Arduino IDE (in Downloads)' && echo '=====================================================' && read -p 'Press [Enter] when flashing is complete to start pneumatic control… ' && cd $REPO/src/devel_packages/orio && ORIO_LOGGING=$LOGGING python3 pneumatic_control_recovery.py"
    SM_ARGS=""
fi

CMD_SM="source $WF && wait_for_service /compute_grasps && docker exec -e ORIO_LOGGING=$LOGGING -it $CONTAINER bash -c 'source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && python3 state_machine.py$SM_ARGS'"

# ── Build the session ───────────────────────────────────────────────────────
session_root "$REPO"

if initialize_session "orio"; then
    # Panes tiled into one window; re-tile between splits so they stay splittable.
    # Pane titles are used by stop_demo.sh to flush the recorder + perception first.
    new_window "orio"
    run_cmd "$CMD_ROSCORE"
    tmux select-pane -t "$session:$window" -T roscore
    PANES=()
    TITLES=()
    if [ "$LOGGING" != "0" ]; then
        PANES+=("$CMD_RECORDER"); TITLES+=(recorder)
    fi
    PANES+=("$CMD_DOC" "$CMD_LUISA" "$CMD_DOCKER" "$CMD_CAMERAS")
    TITLES+=(doc luisa docker cameras)
    if [ -n "$DEXNET" ]; then
        PANES+=("$CMD_DEXNET"); TITLES+=(dexnet)
    fi
    PANES+=("$CMD_PERCEPTION" "$CMD_PNEU" "$CMD_SM")
    TITLES+=(perception pneumatics state_machine)
    for i in "${!PANES[@]}"; do
        split_v
        tmux select-layout -t "$session:$window" tiled
        run_cmd "${PANES[$i]}"
        tmux select-pane -t "$session:$window" -T "${TITLES[$i]}"
    done
    tmux select-layout -t "$session:$window" tiled
    select_window orio
fi

finalize_and_go_to_session
