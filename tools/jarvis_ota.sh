#!/bin/bash
# Jarvis'e Wi-Fi uzerinden firmware yukler (konsol portu 23, 'ota' komutu).
# Kullanim: tools/jarvis_ota.sh [robot-ip] [--build]
#   --build : once idf.py build calistirir
set -e
ROBOT_IP="192.168.1.125"
BUILD=0
for arg in "$@"; do
    case "$arg" in
        --build) BUILD=1 ;;
        *) ROBOT_IP="$arg" ;;
    esac
done
PORT=8070
cd "$(dirname "$0")/.."

if [ "$BUILD" = 1 ]; then
    source "$HOME/esp/esp-idf/export.sh" >/dev/null 2>&1
    idf.py build | tail -3
fi
[ -f build/xiaozhi.bin ] || { echo "build/xiaozhi.bin yok: once derleyin (--build)"; exit 1; }

MAC_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null)
[ -n "$MAC_IP" ] || { echo "Mac'in Wi-Fi IP adresi bulunamadi"; exit 1; }
nc -z -w 3 "$ROBOT_IP" 23 || { echo "Jarvis konsoluna ulasilamadi: $ROBOT_IP:23"; exit 1; }

python3 -m http.server "$PORT" --bind "$MAC_IP" --directory build >/tmp/jarvis_ota_http.log 2>&1 &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT
sleep 1

echo "Yukleniyor: http://$MAC_IP:$PORT/xiaozhi.bin ($(du -h build/xiaozhi.bin | cut -f1)) -> $ROBOT_IP"
# Komutu gonder, ilerlemeyi goster; "rebooting"/"failed" gorunce ya da baglanti kopunca bitir.
python3 - "$ROBOT_IP" "http://$MAC_IP:$PORT/xiaozhi.bin" <<'PY'
import socket, sys, time
ip, url = sys.argv[1], sys.argv[2]
s = socket.create_connection((ip, 23), timeout=5)
s.sendall(f"ota {url}\n".encode())
s.settimeout(1)
end, buf = time.time() + 150, b""
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
        if any(k in line for k in ("ota ", "Progress", "pgrade", "FAILED", "rebooting")):
            print(line, flush=True)
        if "rebooting" in line or "FAILED" in line or "failed" in line:
            sys.exit(0)
PY
echo "Robotun acilmasi bekleniyor..."
for i in $(seq 1 40); do
    sleep 2
    if nc -z -w 2 "$ROBOT_IP" 23 2>/dev/null; then echo "Jarvis yeni firmware ile acildi."; exit 0; fi
done
echo "Jarvis 80 sn icinde konsola donmedi; seri logu kontrol edin."
