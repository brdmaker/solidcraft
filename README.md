# Built binaries (orphan branch)

No source here; this branch holds build outputs only. Source is on `main`.

| Path | What |
|---|---|
| `linux-x86_64/solidcraft-cli.gz` | `solidcraft-cli` 0.27.1, release build, stripped, gzip -9 (unpacks to about 81 MB) |
| `linux-x86_64/solvecraft-cli.gz` | `solvecraft-cli`, same build |

Built from `main` commit 6e4e94a with `cargo build --release -p solidcraft-cli`.
Use: `gunzip -k linux-x86_64/solidcraft-cli.gz && chmod +x linux-x86_64/solidcraft-cli`; verify with `SHA256SUMS` (hashes of the .gz files).
Used to generate the Scripta CN 720 retrofit parts in the 3dassets repo.
