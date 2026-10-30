# Persistent MCP session: serves tool calls over a unix socket (one JSON request per connection).
import base64, json, os, socket, subprocess, sys
bin_, sock, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
p = subprocess.Popen([bin_, "mcp", "--headless", "--allow-path", outdir], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
n = 0
def rpc(method, params, notify=False):
    global n
    msg = {"jsonrpc": "2.0", "method": method, "params": params}
    if not notify: n += 1; msg["id"] = n
    p.stdin.write(json.dumps(msg) + "\n"); p.stdin.flush()
    if notify: return None
    while True:
        r = json.loads(p.stdout.readline())
        if r.get("id") == n: return r
rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "d", "version": "0"}})
rpc("notifications/initialized", {}, notify=True)
if os.path.exists(sock): os.unlink(sock)
s = socket.socket(socket.AF_UNIX); s.bind(sock); s.listen(1)
shot = 0
while True:
    c, _ = s.accept(); data = b""
    while not data.endswith(b"\n"):
        chunk = c.recv(65536)
        if not chunk: break
        data += chunk
    req = json.loads(data)
    r = rpc("tools/call", {"name": req["tool"], "arguments": req.get("args", {})})
    res = r.get("result", r.get("error")); out = []
    if isinstance(res, dict):
        if res.get("isError"): out.append("ERROR")
        for it in res.get("content", []):
            if it.get("type") == "image":
                shot += 1; path = f"{outdir}/img{shot}.png"; open(path, "wb").write(base64.b64decode(it["data"])); out.append("image -> " + path)
            else: out.append(it.get("text", ""))
    else: out.append(json.dumps(r))
    c.sendall("\n".join(out).encode()); c.close()
