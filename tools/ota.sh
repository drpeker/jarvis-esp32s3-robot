#!/bin/bash
# Update Jarvis over Wi-Fi: serves build/jarvis.bin from this Mac and sends the robot
# console (port 23) the 'ota' command. Works from the old Jarvis firmware too (same command).
# Usage: tools/ota.sh [robot-ip] [--build]
set -e
ROBOT_IP="192.168.1.125"
BUILD=0
for arg in "$@"; do
    case "$arg" in
        --build) BUILD=1 ;;
        *) ROBOT_IP="$arg" ;;
    esac
done
PORT=8071
cd "$(dirname "$0")/.."
if [ "$BUILD" = 1 ]; then
    source "$HOME/esp/esp-idf/export.sh" >/dev/null 2>&1
    idf.py build | tail -2
fi
BIN=build/jarvis.bin
[ -f "$BIN" ] || { echo "$BIN missing: build first (--build)"; exit 1; }
MAC_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null)
[ -n "$MAC_IP" ] || { echo "no Wi-Fi IP on this Mac"; exit 1; }
nc -z -w 3 "$ROBOT_IP" 23 || { echo "robot console not reachable: $ROBOT_IP:23 (close other consoles)"; exit 1; }
python3 -m http.server "$PORT" --bind "$MAC_IP" --directory build >/tmp/jarvis_v17_ota.log 2>&1 &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT
sleep 1
echo "sending http://$MAC_IP:$PORT/jarvis.bin ($(du -h $BIN | cut -f1)) to $ROBOT_IP"
python3 - "$ROBOT_IP" "http://$MAC_IP:$PORT/jarvis.bin" <<'PY'
import socket, sys, time
ip, url = sys.argv[1], sys.argv[2]
s = socket.create_connection((ip, 23), timeout=5)
s.sendall(f"ota {url}\n".encode())
s.settimeout(1)
end, buf = time.time() + 180, b""
while time.time() < end:
    try:
        d = s.recv(4096)
    except socket.timeout:
        continue
    except OSError:
        break
    if not d:
        break
    buf += d
    *lines, buf = buf.split(b"\n")
    for raw in lines:
        line = raw.decode(errors="replace").strip()
        if "OTA" in line or "rogress" in line or "pgrade" in line or "ailed" in line or "reboot" in line:
            print(line, flush=True)
        if "reboot" in line or "FAILED" in line or "failed" in line:
            sys.exit(0)
PY
echo "waiting for the robot..."
for i in $(seq 1 40); do
    sleep 2
    if nc -z -w 2 "$ROBOT_IP" 23 2>/dev/null; then echo "robot is back"; exit 0; fi
done
echo "robot did not come back within 80 s"
exit 1
