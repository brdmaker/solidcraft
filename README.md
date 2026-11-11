# solidcraft-cli build of 2026-11-11 (orphan branch)

No source here; this branch holds build outputs and a copy of the agent skills that apply when
using the CLI. Source is on `main`.

| Path | What |
|---|---|
| `linux-x86_64/solidcraft-cli.gz` | `solidcraft-cli` 0.40.0-rc.4, release build, stripped, gzip -9 (unpacks to about 154 MB) |
| `linux-x86_64/solvecraft-cli.gz` | `solvecraft-cli`, the old-name alias that forwards to `solidcraft-cli`, same build |
| `linux-x86_64/SHA256SUMS` | hashes of the .gz files |
| `skills/` | agent skills copied from `main`'s `skills/` at the same commit (see below) |

Built from `main` commit 525171eb with `cargo build --release -p solidcraft-cli`
(Rust 1.98.0 from `rust-toolchain.toml`, default features), then `strip` and `gzip -9 -n`.

Use: `gunzip -k linux-x86_64/solidcraft-cli.gz && chmod +x linux-x86_64/solidcraft-cli`;
verify with `cd linux-x86_64 && sha256sum -c SHA256SUMS`. Checked: `--version` prints
0.40.0-rc.4, and `eval` of `ballscrew-nuts-sfk/SFK1204/sfk1204.json` from the 3dassets repo
gives 13309.95 mm³ (expected 13310.0).

## Skills

| Skill | Use it when |
|---|---|
| `solidcraft-modeling` | building a new part |
| `solidcraft-editing` | changing or fixing an existing design |
| `solidcraft-checking` | checking parameter ranges, comparing STEP exactly, designs in git, porting (has the `run.py`/`compare_step.py` scripts) |
| `solidcraft-mechanisms` | assemblies, joints, motion |
| `solidcraft-sheetmetal` | sheet metal parts, flat patterns, DXF |
| `solidcraft-reporting` | a crash or wrong result to report upstream |
| `solidcraft-agent-tools` | running SolidCraft, the CLI and the control channel |

The first five are the modelling skills SolidCraft also builds into its MCP server
(`skill_reference`). Their examples use MCP tool calls; the same commands go in a CLI script
(`solidcraft-cli run script.json`). `skills/README.upstream.md` is the upstream index of all
skills; the contributor-only skills are not copied here.
