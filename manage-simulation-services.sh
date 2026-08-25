#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"; nolo=""
start(){ cargo build --release --manifest-path "$root/vr-xr/src/nolo-usb-server/Cargo.toml"; "$root/vr-xr/src/nolo-usb-server/target/release/nolo-usb-server" --host=192.168.100.10 --port=8765 & nolo=$!; trap 'kill "$nolo" 2>/dev/null || true' EXIT INT TERM; "$root/arm/ros2/run-moveit-simulation.sh"; }
stop(){ docker stop stararm102-moveit-simulation 2>/dev/null || true; pkill -TERM -x nolo-usb-server 2>/dev/null || true; }
case "${1:-}" in
  start) start;; stop) stop;; restart) stop; start;;
  *) echo "用法：$0 start|restart|stop" >&2; exit 2;;
esac
