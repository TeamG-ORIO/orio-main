# tmuxifier session layout for the ORIO demo: 8 readiness-gated panes in one
# tiled window. Launched by launch_demo.sh (exports the ORIO_* env vars).

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

# ── Per-service commands (readiness-gated) ──────────────────────────────────
CMD_ROSCORE="roscore"

CMD_DOC="cd $FRANKAPY && source $WF && wait_for_roscore && bash ./bash_scripts/start_control_pc.sh -u student -i iam-doc"
CMD_SLEEPY="cd $FRANKAPY && source $WF && wait_for_roscore && bash ./bash_scripts/start_control_pc_sleepy.sh -u iam-sleepy -i iamsleepy"

CMD_DOCKER="cd $REPO && source $WF && wait_for_roscore && bash orio_run_docker.sh"

CMD_CAMERAS="source $WF && wait_for_container $CONTAINER && docker exec -it $CONTAINER bash -c 'source /home/ros_ws/devel/setup.bash && roslaunch manipulation cameras.launch'"

CMD_PERCEPTION="source $WF && wait_for_topic /camera/rgb/image_raw && wait_for_topic /zedm/zed_node/rgb/image_rect_color && source $PERC_VENV/bin/activate && ORIO_REPO=$REPO ORIO_PERCEPTION_ASSETS=$PERC_ASSETS ORIO_GROUNDINGDINO_DIR=$GDINO_DIR python3 $PERC_SCRIPTS/perception_control_combined_pass_through.py"

if [ -n "$NO_VACUUM" ]; then
    CMD_PNEU="echo '[pneumatics] DISABLED (dry-run --no-vacuum): not starting pneumatic_control. Move the cup by hand; vacuum commands and sensor checks are skipped in the state machine.'"
    SM_ARGS=" --no-vacuum"
else
    CMD_PNEU="echo '=====================================================' && echo ' ACTION REQUIRED: flash the vacuum firmware via Arduino IDE (in Downloads)' && echo '=====================================================' && read -p 'Press [Enter] when flashing is complete to start pneumatic control… ' && cd $REPO/src/devel_packages/orio && python3 pneumatic_control_recovery.py"
    SM_ARGS=""
fi

CMD_SM="source $WF && wait_for_service /compute_grasps && docker exec -it $CONTAINER bash -c 'source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && python3 state_machine.py$SM_ARGS'"

# ── Build the session ───────────────────────────────────────────────────────
session_root "$REPO"

if initialize_session "orio"; then
    # 8 panes tiled into one window; re-tile between splits so they stay splittable.
    new_window "orio"
    run_cmd "$CMD_ROSCORE"
    for cmd in "$CMD_DOC" "$CMD_SLEEPY" "$CMD_DOCKER" "$CMD_CAMERAS" \
               "$CMD_PERCEPTION" "$CMD_PNEU" "$CMD_SM"; do
        split_v
        tmux select-layout -t "$session:$window" tiled
        run_cmd "$cmd"
    done
    tmux select-layout -t "$session:$window" tiled
    select_window orio
fi

finalize_and_go_to_session
