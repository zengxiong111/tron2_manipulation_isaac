#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 ]]; then
    echo "usage: $0 UPSTREAM_DIR" >&2
    exit 2
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
upstream_dir="$(realpath -m -- "$1")"
case "$upstream_dir/" in
    "$repo_dir/"*) echo "Choose an upstream directory outside this repository." >&2; exit 2 ;;
esac
controller_rev=10af63c20e0dd2149168c6693453cc28f2c2dcbf
description_rev=9939c22e69d27653ec0ba8a505859a2903dd1a71

if [[ ! -e "$upstream_dir" ]]; then
    git clone --no-checkout --depth 1 https://github.com/limxdynamics/tron2_manipulation_controller.git "$upstream_dir"
    git -C "$upstream_dir" fetch --depth 1 origin "$controller_rev"
    git -C "$upstream_dir" checkout --detach "$controller_rev"
fi
if [[ ! -d "$upstream_dir/.git" || "$(git -C "$upstream_dir" rev-parse HEAD)" != "$controller_rev" ]]; then
    echo "Expected upstream commit $controller_rev in $upstream_dir. Use a new directory to fetch it." >&2
    exit 1
fi
git -C "$upstream_dir" submodule update --init --depth 1 robot_description
actual_description="$(git -C "$upstream_dir/robot_description" rev-parse HEAD)"
[[ "$actual_description" == "$description_rev" ]]
printf 'Controller   %s\nDescription  %s\nPath         %s\n' "$controller_rev" "$description_rev" "$upstream_dir"
