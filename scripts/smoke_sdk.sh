#!/usr/bin/env bash
set -euo pipefail

# Verify only a WebSocket HTTP upgrade. Never sends a motion request.
if [[ $# -ne 1 ]]; then
  echo "usage: $0 ws://127.0.0.1:PORT" >&2
  exit 2
fi
url="$1"
python3 - "$url" <<'PY'
import base64
import os
import socket
import sys
from urllib.parse import urlsplit

url = urlsplit(sys.argv[1])
if url.scheme != "ws" or url.hostname not in {"127.0.0.1", "localhost", "::1"}:
    raise SystemExit("Refusing non-local or non-ws endpoint; use an isolated local controller.")
if not url.port:
    raise SystemExit("WebSocket URL must include an explicit port.")
key = base64.b64encode(os.urandom(16)).decode("ascii")
path = url.path or "/"
if url.query:
    path += "?" + url.query
sock = socket.create_connection((url.hostname, url.port), timeout=3)
with sock:
    request = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {url.hostname}:{url.port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(request.encode("ascii"))
    response = sock.recv(4096).decode("latin1", errors="replace")
if not response.startswith("HTTP/1.1 101") and not response.startswith("HTTP/1.0 101"):
    raise SystemExit("WebSocket upgrade failed; response was: " + response.splitlines()[0])
print(f"WebSocket upgrade accepted at {url.hostname}:{url.port}; no motion command sent.")
PY
