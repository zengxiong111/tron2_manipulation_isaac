#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
engine=""
bridge_install=""
output=""
seconds=30
expected_rate=300
stale_after_ms=20
startup_timeout=30
observer_mode=bridged
command=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --engine) engine=${2:?missing engine}; shift 2 ;;
        --bridge-install) bridge_install=${2:?missing bridge install}; shift 2 ;;
        --output) output=${2:?missing output path}; shift 2 ;;
        --seconds) seconds=${2:?missing duration}; shift 2 ;;
        --expected-rate-hz) expected_rate=${2:?missing expected rate}; shift 2 ;;
        --stale-after-ms) stale_after_ms=${2:?missing stale threshold}; shift 2 ;;
        --startup-timeout-s) startup_timeout=${2:?missing startup timeout}; shift 2 ;;
        --direct-ros2) observer_mode=direct; shift ;;
        --) shift; command=("$@"); break ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [[ "$engine" != mujoco && "$engine" != isaac ]]; then
    echo "usage: $0 --engine mujoco|isaac --bridge-install PATH --output OUTSIDE_REPO.jsonl [--direct-ros2] [--seconds N] [-- COMMAND ...]" >&2
    exit 2
fi
if [[ -z "$output" || -z "$bridge_install" ]]; then
    echo "--output and --bridge-install are required; --direct-ros2 skips only the observer bridge process" >&2
    exit 2
fi

if [[ ! "$startup_timeout" =~ ^[1-9][0-9]{0,3}$ ]] || (( startup_timeout > 3600 )); then
    echo "--startup-timeout-s must be an integer from 1 through 3600" >&2
    exit 2
fi

if [[ -n "$bridge_install" ]]; then
    bridge_install="$(realpath -e -- "$bridge_install")"
fi
output="$(realpath -m -- "$output")"
case "$output" in
    "$repo_dir"|"$repo_dir"/*)
        echo "timing output must be outside the repository" >&2
        exit 2
        ;;
esac
if [[ -e "$output" ]]; then
    echo "refusing to overwrite timing output: $output" >&2
    exit 2
fi
ready_file="$output.ready"
if [[ -e "$ready_file" ]]; then
    echo "refusing to overwrite timing ready marker: $ready_file" >&2
    exit 2
fi
mkdir -p "$(dirname -- "$output")"

ros_setup=/opt/ros/jazzy/setup.bash
if [[ ! -f "$ros_setup" ]]; then
    echo "ROS 2 Jazzy setup file is missing." >&2
    exit 1
fi
if [[ ! -f "$bridge_install/setup.bash" ]]; then
    echo "ros2-bridger setup file is missing." >&2
    exit 1
fi

# Source the same public Jazzy bundle in this read-only observer domain.
set +u
source "$ros_setup"
source "$bridge_install/setup.bash"
set -u

main_domain="${ROS_DOMAIN_ID:-0}"
if [[ ! "$main_domain" =~ ^[0-9]{1,3}$ ]]; then
    echo "ROS_DOMAIN_ID must be an integer from 0 through 232" >&2
    exit 2
fi
main_domain_num=$((10#$main_domain))
if (( main_domain_num > 232 )); then
    echo "ROS_DOMAIN_ID must be an integer from 0 through 232" >&2
    exit 2
fi
observer_boundary="MROS DDS topic -> observer ros2-bridger -> ROS 2 subscription callback"
if [[ "$observer_mode" == bridged ]]; then
    domain="${TRON2_TIMING_ROS_DOMAIN_ID:-77}"
    if [[ ! "$domain" =~ ^[0-9]{1,3}$ ]]; then
        echo "TRON2_TIMING_ROS_DOMAIN_ID must be an integer from 0 through 232" >&2
        exit 2
    fi
    domain_num=$((10#$domain))
    if (( domain_num > 232 )); then
        echo "TRON2_TIMING_ROS_DOMAIN_ID must be an integer from 0 through 232" >&2
        exit 2
    fi
    if (( domain_num == main_domain_num )); then
        echo "observer ROS domain $domain_num matches the main bridge domain; set TRON2_TIMING_ROS_DOMAIN_ID to a different domain" >&2
        exit 2
    fi
    export ROS_DOMAIN_ID="$domain_num"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export MROS_IP_LIST=127.0.0.1
    export MROS_LOCALHOST_ONLY=0
else
    observer_boundary="/motor/cmd after the primary MROS-to-ROS 2 bridge; /motor/state at the Isaac sidecar ROS 2 publication"
fi
observer_python=/usr/bin/python3
if [[ ! -x "$observer_python" ]]; then
    echo "ROS observer Python is unavailable: $observer_python" >&2
    exit 1
fi
bridge_log="$output.bridge.log"
bridge_pid=""
observer_pid=""
ready_owned=0
cleanup() {
    if [[ -n "$observer_pid" ]]; then
        kill "$observer_pid" 2>/dev/null || true
        wait "$observer_pid" 2>/dev/null || true
    fi
    if [[ -n "$bridge_pid" ]]; then
        kill -- "-$bridge_pid" 2>/dev/null || kill "$bridge_pid" 2>/dev/null || true
        wait "$bridge_pid" 2>/dev/null || true
    fi
    if (( ready_owned )); then
        rm -f "$ready_file"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "$observer_mode" == bridged ]]; then
    setsid ros2 launch mrosbridger mrosbridger.launch.py \
        bridge_ros2mros:=false \
        'mros2ros_include:=/motor/cmd;/motor/state' \
        >"$bridge_log" 2>&1 &
    bridge_pid=$!
fi

"$observer_python" -u "$repo_dir/scripts/mros_timing_observer.py" \
    --engine "$engine" \
    --output "$output" \
    --seconds "$seconds" \
    --expected-rate-hz "$expected_rate" \
    --stale-after-ms "$stale_after_ms" \
    --startup-timeout-s "$startup_timeout" \
    --observer-boundary "$observer_boundary" \
    --ready-file "$ready_file" &
observer_pid=$!

ready_deadline=$((SECONDS + startup_timeout + 10))
while [[ ! -f "$ready_file" ]] && kill -0 "$observer_pid" 2>/dev/null && (( SECONDS < ready_deadline )); do
    sleep 0.1
done
if [[ ! -f "$ready_file" ]]; then
    if [[ "$observer_mode" == bridged ]]; then
        echo "Timing observer failed to see both motor topics. Bridge log: $bridge_log" >&2
    else
        echo "Timing observer failed to see both motor topics in ROS_DOMAIN_ID=$main_domain." >&2
    fi
    exit 1
fi
ready_owned=1

command_status=0
if [[ ${#command[@]} -gt 0 ]]; then
    "${command[@]}" || command_status=$?
fi
observer_status=0
wait "$observer_pid" || observer_status=$?
observer_pid=""
if (( command_status != 0 )); then
    exit "$command_status"
fi
exit "$observer_status"
