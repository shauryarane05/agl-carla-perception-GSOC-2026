#!/usr/bin/env python3
"""Stdlib rosbridge client for AGL: subscribe to PCL detections (Detection3DArray).
Usage: python3 agl_lidar_test.py 10.0.2.2 9090 /carla/detections"""
import socket, os, base64, struct, json, sys

def ws_connect(host, port):
    s = socket.create_connection((host, port), timeout=10)
    key = base64.b64encode(os.urandom(16)).decode()
    req = ("GET / HTTP/1.1\r\n" f"Host: {host}:{port}\r\n"
           "Upgrade: websocket\r\nConnection: Upgrade\r\n"
           f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
    s.sendall(req.encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        c = s.recv(1)
        if not c: raise RuntimeError("closed during handshake")
        buf += c
    if b"101" not in buf.split(b"\r\n", 1)[0]:
        raise RuntimeError("handshake failed")
    return s

def ws_send_text(s, text):
    p = text.encode(); h = bytearray([0x81]); n = len(p); mb = 0x80
    if n < 126: h.append(mb | n)
    elif n < 65536: h.append(mb | 126); h += struct.pack(">H", n)
    else: h.append(mb | 127); h += struct.pack(">Q", n)
    m = os.urandom(4); h += m
    s.sendall(bytes(h) + bytes(b ^ m[i % 4] for i, b in enumerate(p)))

def _rx(s, n):
    d = b""
    while len(d) < n:
        c = s.recv(n - len(d))
        if not c: raise RuntimeError("closed")
        d += c
    return d

def ws_recv_text(s):
    while True:
        b0, b1 = _rx(s, 2); op = b0 & 0x0F; mk = b1 & 0x80; ln = b1 & 0x7F
        if ln == 126: ln = struct.unpack(">H", _rx(s, 2))[0]
        elif ln == 127: ln = struct.unpack(">Q", _rx(s, 8))[0]
        mask = _rx(s, 4) if mk else None
        pl = _rx(s, ln)
        if mask: pl = bytes(b ^ mask[i % 4] for i, b in enumerate(pl))
        if op == 0x8: raise RuntimeError("server closed")
        if op in (0x1, 0x2): return pl.decode(errors="replace")

def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "10.0.2.2"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9090
    topic = sys.argv[3] if len(sys.argv) > 3 else "/carla/detections"
    print("[agl] connecting to ws://%s:%d topic=%s" % (host, port, topic))
    s = ws_connect(host, port)
    print("[agl] handshake OK, subscribing...")
    ws_send_text(s, json.dumps({"op": "subscribe", "topic": topic}))
    count = 0
    while True:
        d = json.loads(ws_recv_text(s))
        if d.get("op") != "publish": continue
        dets = d.get("msg", {}).get("detections", [])
        count += 1
        line = "[agl] #%d %s  %d detections" % (count, topic, len(dets))
        if dets:
            b = dets[0]["bbox"]; c = b["center"]["position"]; sz = b["size"]
            line += "  | box0 center=(%.1f,%.1f,%.1f) size=(%.1f,%.1f,%.1f)" % (
                c["x"], c["y"], c["z"], sz["x"], sz["y"], sz["z"])
        print(line)

if __name__ == "__main__":
    main()
