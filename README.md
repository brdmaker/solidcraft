# Prebuilt solidcraft-cli (Linux x86_64)

This orphan branch holds only a compiled binary, so a session can skip the ~11 minute release build. It shares no history with `main`.

| | |
|---|---|
| Version | solidcraft-cli 0.27.1 |
| Source commit | `6e4e94a8a86418fd91aef94f8e3c928a8465be65` (main) |
| Built | 2026-10-30, `cargo build --release -p solidcraft-cli`, rustc 1.97.0, then `strip` |
| Target | x86_64-unknown-linux-gnu, needs glibc >= 2.39 |

The binary is gzipped (81.6 MB unpacked) to stay under GitHub's file-size limits. Checksums for both are in `SHA256SUMS`.

## Use

```sh
git fetch origin prebuilt/solidcraft-cli-linux-x86_64
git show origin/prebuilt/solidcraft-cli-linux-x86_64:solidcraft-cli.gz | gunzip > solidcraft-cli
chmod +x solidcraft-cli
./solidcraft-cli --version
./solidcraft-cli mcp --headless          # MCP server over stdio
```

`mcp-client.py` is a minimal stdio MCP client that runs a JSON list of tool calls
(`[{"tool":"measure","args":{}}]`) in one session:
`python3 -I mcp-client.py ./solidcraft-cli plan.json OUTDIR`.

This binary is stale as soon as `main` moves; rebuild from source when you need current behaviour.
