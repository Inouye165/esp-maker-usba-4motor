#!/usr/bin/env bash
# run_gold_standard_home_test.sh - Canonical Entrypoint for Gold Standard HOME Acceptance Test
# Strictly adheres to docs/EXACT_MOTION_CONTRACT.md and rover-exact-motion skill.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Verify if running directly on Pi host where docker container exists
if command -v docker >/dev/null 2>&1 && sudo docker ps --format '{{.Names}}' | grep -q "^rover-ros2$"; then
    echo "[LAUNCHER] Executing Gold Standard Acceptance Test inside rover-ros2 container..."
    ENV_ARG=""
    if [ -f "/home/ron/yahboom-encoder/.env" ]; then
        ENV_ARG="--env-file /home/ron/yahboom-encoder/.env"
    elif [ -f "${REPO_DIR}/.env" ]; then
        ENV_ARG="--env-file ${REPO_DIR}/.env"
    fi

    DOCKER_TTY_ARG="-i"
    if [ -t 0 ]; then
        DOCKER_TTY_ARG="-it"
    fi

    sudo docker exec ${DOCKER_TTY_ARG} ${ENV_ARG} rover-ros2 bash -c \
        "source /opt/ros/jazzy/setup.bash && export PYTHONPATH=\"/ros2_ws:\${PYTHONPATH:-}\" && python3 -m tools.gold_standard_home_test.cli $*"
else
    # Host / workstation python execution (supports unit test, dry-run, and test-safety)
    echo "[LAUNCHER] Executing Gold Standard Acceptance Test on host Python..."
    export PYTHONPATH="${REPO_DIR}:${PYTHONPATH:-}"
    python3 -m tools.gold_standard_home_test.cli "$@"
fi
