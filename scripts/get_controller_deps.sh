#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 || $(dpkg --print-architecture) != amd64 ]]; then
    echo "usage: $0 EXTERNAL_RUNTIME_DIR (Ubuntu amd64)" >&2
    exit 2
fi
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
runtime_dir="$(realpath -m -- "$1")"
case "$runtime_dir/" in
    "$repo_dir/"*) echo "Choose a runtime directory outside this repository." >&2; exit 2 ;;
esac

work_dir="$(mktemp -d)"
trap 'rm -rf -- "$work_dir"' EXIT
mkdir -p "$work_dir"/{lists,cache,debs} "$runtime_dir"
cat > "$work_dir/focal.list" <<'SOURCES'
deb [arch=amd64 signed-by=/usr/share/keyrings/ubuntu-archive-keyring.gpg] https://archive.ubuntu.com/ubuntu focal main universe
deb [arch=amd64 signed-by=/usr/share/keyrings/ubuntu-archive-keyring.gpg] https://archive.ubuntu.com/ubuntu focal-updates main universe
SOURCES
apt_options=(
    -o "Dir::Etc::sourcelist=$work_dir/focal.list"
    -o Dir::Etc::sourceparts=-
    -o "Dir::State::lists=$work_dir/lists"
    -o "Dir::Cache=$work_dir/cache"
    -o APT::Get::List-Cleanup=0
)
apt-get "${apt_options[@]}" update
(
    cd "$work_dir/debs"
    apt-get "${apt_options[@]}" download libwebsockets15=3.2.1-3 libssl1.1=1.1.1f-1ubuntu2.24 libev4=1:4.31-1
    for package in ./*.deb; do
        dpkg-deb --extract "$package" "$runtime_dir"
    done
)
printf 'TRON2_EXTRA_LIB_DIR=%s/usr/lib/x86_64-linux-gnu\n' "$runtime_dir"
