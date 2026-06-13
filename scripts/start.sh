# start.sh - one-click UAV system startup
# Usage:  make start  (from ~/uav_ws)
# Detach: Ctrl+B D    Reattach: make attach
#
# Window layout:
#   0: dashboard   - live health + status (default focus)
#   1: controller  - most important logs during flight
#   2: fusion      - fusion quality and output
#   3: logger      - CSV logging (background)
#   4: bg          - dmux + other background processes (hidden, auto-scroll off)
#
# Boot order: RPi (systemd starts DDS agent) → Pixhawk (DDS client connects)
# DDS agent service: sudo systemctl status uxrce-dds

SESSION="uav"
WS="source ~/ros2_ws/install/setup.bash"
S="~/ros2_ws/src/optical_flow_fusion/scripts"

tmux kill-session -t $SESSION 2>/dev/null

# Window 0: dashboard - starts last (needs other nodes up first)
tmux new-session  -d -s $SESSION -n dashboard \
    "$WS && sleep 3 && python3 $S/dashboard_node.py; bash"

# Window 1: controller - second most important to watch
tmux new-window -t $SESSION -n controller \
    "$WS && sleep 2 && python3 $S/controller_node.py; bash"

# Window 2: fusion - shows quality stats every 5s
tmux new-window -t $SESSION -n fusion \
    "$WS && sleep 1 && python3 $S/fusion_node.py; bash"

# Window 3: logger - silent background logger
tmux new-window -t $SESSION -n logger \
    "$WS && sleep 2 && python3 $S/logger_node.py; bash"

# Window 4: background - dmux (no output after startup)
tmux new-window -t $SESSION -n bg \
    "$WS && python3 $S/dmux_node.py; bash"

# Focus dashboard on attach
tmux select-window -t $SESSION:0

echo ""
echo "  UAV session started: '$SESSION'"
echo "  ─────────────────────────────────────"
echo "  Ctrl+B 0  → dashboard  (health + status)"
echo "  Ctrl+B 1  → controller (flight commands)"
echo "  Ctrl+B 2  → fusion     (sensor quality)"
echo "  Ctrl+B 3  → logger     (CSV recording)"
echo "  Ctrl+B 4  → bg         (dmux)"
echo "  Ctrl+B D  → detach"
echo ""
tmux attach -t $SESSION
