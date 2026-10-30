# Minimal MCP stdio client: runs a list of tool calls in one server session.
import base64, json, subprocess, sys
bin_, plan, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
calls = json.load(open(plan))
p = subprocess.Popen([bin_, "mcp", "--headless", "--allow-path", outdir], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
n = 0
def rpc(method, params, notify=False):
    global n
    msg = {"jsonrpc": "2.0", "method": method, "params": params}
    if not notify:
        n += 1; msg["id"] = n
    p.stdin.write(json.dumps(msg) + "\n"); p.stdin.flush()
    if notify: return None
    while True:
        r = json.loads(p.stdout.readline())
        if r.get("id") == n: return r
init = rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "demo", "version": "0"}})
print("SERVER:", init["result"]["serverInfo"], "|", init["result"].get("instructions", "")[-80:])
rpc("notifications/initialized", {}, notify=True)
for i, c in enumerate(calls):
    r = rpc("tools/call", {"name": c["tool"], "arguments": c.get("args", {})})
    res = r.get("result", r.get("error"))
    print(f"\n=== {i} {c['tool']} isError={res.get('isError') if isinstance(res, dict) else '?'}")
    for item in (res.get("content", []) if isinstance(res, dict) else []):
        if item.get("type") == "image":
            path = f"{outdir}/shot{i}.png"; open(path, "wb").write(base64.b64decode(item["data"])); print("image ->", path)
        else:
            t = item.get("text", ""); print(t[: c.get("max", 1500)])
    if not isinstance(res, dict) or res.get("isError"): break
p.stdin.close(); p.wait()
