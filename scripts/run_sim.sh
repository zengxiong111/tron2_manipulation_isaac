#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" != --inside ]]; then
    : "${TRON2_UPSTREAM:?Set TRON2_UPSTREAM to the pinned controller checkout}"
    : "${TRON2_BRIDGE_INSTALL:?Set TRON2_BRIDGE_INSTALL to the pinned Jazzy bridge install}"
    : "${ISAAC_PYTHON:?Set ISAAC_PYTHON to the Isaac Sim Python launcher}"
    export TRON2_UPSTREAM="$(realpath -e -- "$TRON2_UPSTREAM")"
    export TRON2_BRIDGE_INSTALL="$(realpath -e -- "$TRON2_BRIDGE_INSTALL")"
    export ISAAC_PYTHON="$(command -v -- "$ISAAC_PYTHON")"
    export TRON2_OUTER_NETWORK="$(readlink /proc/self/ns/net)"
    exec unshare --user --map-root-user --net --mount --fork --kill-child \
        "$repo_dir/scripts/run_sim.sh" --inside "$@"
fi
shift
if [[ -z "${TRON2_OUTER_NETWORK:-}" || $(readlink /proc/self/ns/net) == "$TRON2_OUTER_NETWORK" ]]; then
    echo "The controller and its clients must share a private network namespace." >&2
    exit 1
fi
isaac_args=()
while [[ "${1:-}" == --gui || "${1:-}" == --no-servop-gravity-compensation ]]; do
    isaac_args+=("$1")
    shift
done
if [[ "${1:-}" == -- ]]; then
    shift
elif [[ $# != 0 ]]; then
    echo "usage: $0 [--gui] [--no-servop-gravity-compensation] [-- CLIENT [ARG ...]]" >&2
    exit 2
fi

mount --make-rprivate /
mount -t tmpfs -o mode=1777,size=256M,nosuid,nodev tmpfs /dev/shm
ip link set lo up
export IS_SIM=1 MROS_IP_LIST=127.0.0.1 MROS_LOCALHOST_ONLY=0 MROS_SIM_TIME=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export TRON2_CONTROLLER_INSTALL="$TRON2_UPSTREAM/Tron2_controller/install"
export TRON2_RUN_DIR="$(mktemp -d /tmp/tron2-isaac-run.XXXXXX)"
export ROS_LOG_DIR="$TRON2_RUN_DIR/ros"
printf 'Run directory %s\n' "$TRON2_RUN_DIR"

jobs=()
cleanup() {
    for pid in "${jobs[@]}"; do
        kill -- "-$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start() {
    local name=$1
    shift
    setsid "$@" >"$TRON2_RUN_DIR/$name.log" 2>&1 &
    jobs+=("$!")
}

wait_for() {
    local name=$1 pid=$2 seconds=$3
    shift 3
    local deadline=$((SECONDS + seconds))
    while kill -0 "$pid" 2>/dev/null && (( SECONDS < deadline )); do
        if ! kill -0 "${jobs[0]}" 2>/dev/null; then
            echo "MROS bridge exited. See $TRON2_RUN_DIR/bridge.log" >&2
            tail -n 20 "$TRON2_RUN_DIR/bridge.log" >&2
            exit 1
        fi
        if "$@" >/dev/null 2>&1; then
            return
        fi
        sleep 0.25
    done
    echo "$name failed to become ready. See $TRON2_RUN_DIR/$name.log" >&2
    tail -n 20 "$TRON2_RUN_DIR/$name.log" >&2
    exit 1
}

cd "$repo_dir"
start bridge "$repo_dir/bridge/run_bridge.sh" "$TRON2_BRIDGE_INSTALL"
start isaac "$ISAAC_PYTHON" -m tron2_isaac --upstream "$TRON2_UPSTREAM" \
    --bridge-install "$TRON2_BRIDGE_INSTALL" "${isaac_args[@]}"
wait_for isaac "${jobs[-1]}" 90 grep -q 'Isaac motor simulation ready' "$TRON2_RUN_DIR/isaac.log"
start controller "$repo_dir/scripts/run_controller.sh" --in-isolated-network
wait_for controller "${jobs[-1]}" 30 "$repo_dir/scripts/smoke_sdk.sh" ws://127.0.0.1:5000
"$TRON2_UPSTREAM/joystick_sim/pub.sh" switch
printf 'LimX SDK is ready at ws://127.0.0.1:5000 in this namespace.\n'

if [[ $# != 0 ]]; then
    "$@"
else
    bash --noprofile --norc -i
fi
