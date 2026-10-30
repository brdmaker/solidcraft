# Usage: call.py TOOL 'JSON args' [maxchars]   or   call.py @file.json [maxchars] (file: {"tool":..,"args":..})
import json, socket, sys
a = sys.argv[1:]
if a[0].startswith("@"): req = json.load(open(a[0][1:])); mx = int(a[1]) if len(a) > 1 else 3000
else: req = {"tool": a[0], "args": json.loads(a[1]) if len(a) > 1 else {}}; mx = int(a[2]) if len(a) > 2 else 3000
s = socket.socket(socket.AF_UNIX); s.connect(__import__("os").path.dirname(__import__("os").path.abspath(sys.argv[0])) + "/mcp.sock")
s.sendall((json.dumps(req) + "\n").encode()); buf = b""
while True:
    d = s.recv(65536)
    if not d: break
    buf += d
t = buf.decode(); print(t if len(t) <= mx else t[:mx] + f"\n...[{len(t)} chars]")
