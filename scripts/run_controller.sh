#!/usr/bin/env bash
set -euo pipefail

# Run the upstream prebuilt controller with isolated loopback and an ephemeral
# SN file. The SN utility shipped upstream writes /opt and chmods that tree;
# this wrapper intentionally avoids it.
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
install_dir="${TRON2_CONTROLLER_INSTALL:-$repo_root/Tron2_controller/install}"
sn="${TRON2_ROBOT_SN:-DACH_TRON2A_001}"
mode="isolated"
if [[ "${1:-}" == "--in-isolated-network" ]]; then
  mode="network-present"
  shift
fi
if [[ $# -gt 0 ]]; then
  echo "usage: $0 [--in-isolated-network]" >&2
  exit 2
fi

if [[ ! -f "$install_dir/local_setup.bash" ]]; then
  echo "Controller install not found: $install_dir" >&2
  exit 1
fi
if [[ -z "$sn" || "$sn" == *$'\n'* ]]; then
  echo "TRON2_ROBOT_SN must be a non-empty single line." >&2
  exit 2
fi
if ! command -v unshare >/dev/null || ! command -v mount >/dev/null; then
  echo "unshare and mount are required for isolated controller startup." >&2
  exit 1
fi

if [[ "$mode" == "isolated" ]]; then
  echo "Starting controller in a private network and mount namespace."
  echo "WebSocket port 5000 will be available only inside this namespace."
else
  echo "Starting controller in a private mount namespace, sharing the caller network namespace."
fi
if [[ -n "${TRON2_EXTRA_LIB_DIR:-}" ]]; then
  export LD_LIBRARY_PATH="$TRON2_EXTRA_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export TRON2_KEEP_EXTERNAL_LD_LIBRARY_PATH=1
fi
if [[ "$mode" == "isolated" ]]; then
  exec unshare --user --map-root-user --net --fork --kill-child \
    bash -euo pipefail -c 'ip link set lo up; exec "$1" --in-isolated-network' _ "$0"
fi

# This mode is for a controller joined to an already-private network namespace.
# It gets its own mount namespace so its ephemeral SN state does not touch host
# /opt, while the network namespace stays shared with Isaac and the SDK client.
exec unshare --user --map-root-user --mount --fork --kill-child \
  bash -euo pipefail -c '
    install_dir="$1"
    sn="$2"
    mount --make-rprivate /
    # This controller uses MROS, not ROS 2. Mask /opt only in this mount
    # namespace so the ephemeral SN config cannot touch host /opt.
    mount -t tmpfs -o mode=755,nosuid,nodev tmpfs /opt
    mkdir -p /opt/limx/imptdata
    chmod 700 /opt/limx/imptdata
    printf "sn: %s\n" "$sn" > /opt/limx/imptdata/sn.yaml
    # Current rbtcommon reads SN from userdata.yaml; keep the legacy helper
    # filename too because upstream setRobotSN still writes sn.yaml.
    printf "sn: %s\n" "$sn" > /opt/limx/imptdata/userdata.yaml
    source "$install_dir/local_setup.bash"
    exec mroslaunch tron2_controllers/tron2a_manipulation_controllers_sim.launch
  ' _ "$install_dir" "$sn"
