# Makefile - UAV pipeline control
# Usage: make <target>
# Run from: ~/uav_ws/

SESSION   = uav
WS_SRC    = ~/ros2_ws/src/optical_flow_fusion/scripts
PX4_DIR   = ~/PX4-Autopilot
BOARD     = px4_fmu-v5_default
GCC_PATH  = /opt/gcc-arm-none-eabi-10.3-2021.10/bin

.PHONY: start stop attach \
        dmux fusion logger monitor \
        dds check \
        build firmware flash \
        logs clean-logs help

# ── Session management ────────────────────────────────────────────────────────

start:
	cd ~/uav_ws && bash start.sh

stop:
	tmux kill-session -t $(SESSION) 2>/dev/null || echo "No session running"

attach:
	tmux attach -t $(SESSION)

# ── Individual nodes ──────────────────────────────────────────────────────────

dmux:
	source ~/ros2_ws/install/setup.bash && \
	python3 $(WS_SRC)/dmux_node.py

fusion:
	source ~/ros2_ws/install/setup.bash && \
	python3 $(WS_SRC)/fusion_node.py

logger:
	source ~/ros2_ws/install/setup.bash && \
	python3 $(WS_SRC)/logger_node.py

monitor:
	source ~/ros2_ws/install/setup.bash && \
	ros2 topic echo /fmu/in/fused_optical_flow

# ── DDS agent ─────────────────────────────────────────────────────────────────

dds:
	sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 -v4

# ── Health checks ─────────────────────────────────────────────────────────────

check:
	@echo "=== DDS agent status ==="
	@systemctl is-active uxrce-dds-agent 2>/dev/null || echo "not a systemd service — check manually"

# ── Build ────────────────────────────────────────────────────────────────────

build:
	cd ~/ros2_ws && colcon build && source install/setup.bash

build-fusion:
	cd ~/ros2_ws && colcon build --packages-select optical_flow_fusion && source install/setup.bash

firmware:
	export PATH=$(GCC_PATH):$$PATH && \
	cd $(PX4_DIR) && make $(BOARD)

flash:
	export PATH=$(GCC_PATH):$$PATH && \
	cd $(PX4_DIR) && make $(BOARD) upload

firmware-clean:
	export PATH=$(GCC_PATH):$$PATH && \
	cd $(PX4_DIR) && make distclean && make $(BOARD)

# ── Logs ────────────────────────────────────────────────────────────────────

logs:
	ls -lh ~/uav_logs/

clean-logs:
	rm -f ~/uav_logs/*.csv && echo "Logs cleared"
