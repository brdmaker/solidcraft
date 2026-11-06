//! Plan a lathe job, write its LinuxCNC program and a drawing of the simulated half section:
//! `cargo run -p solidcraft-cam --example lathe -- JOB.json OUT.ngc [OUT.svg]`. The part's
//! `design` is a STEP file relative to the job. Prints the operations, the warnings and the
//! simulation's report.

use std::path::Path;
use std::process::ExitCode;

use solidcraft_cam::lathe::{job::LatheJob, plan, post, profile, sim};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("lathe: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(job_path), Some(out)) = (args.first(), args.get(1)) else {
        return Err("usage: lathe JOB.json OUT.ngc [OUT.svg]".into());
    };
    let base = Path::new(job_path).parent().unwrap_or(Path::new("."));
    let text = std::fs::read_to_string(job_path).map_err(|e| format!("{job_path}: {e}"))?;
    let load = |lib: &str| std::fs::read_to_string(base.join(lib)).map_err(|e| solidcraft_cam::CamError::Invalid(format!("{lib}: {e}")));
    let job = LatheJob::from_json_with(&text, &load).map_err(|e| e.to_string())?;
    let file = base.join(&job.part.design);
    let step = std::fs::read_to_string(&file).map_err(|e| format!("{}: {e}", file.display()))?;
    let read = solidcraft_kernel::step_import(&step).map_err(|e| e.to_string())?;
    let body = read.bodies.iter().find(|b| b.name == job.part.body).or(read.bodies.first()).ok_or("no solid in the STEP file")?;
    let prof = profile::read(&body.body, job.part.front).map_err(|e| e.to_string())?;
    let pl = plan::plan(&job, prof).map_err(|e| e.to_string())?;
    let title = if job.part.name.is_empty() { job.part.body.clone() } else { job.part.name.clone() };
    let program = post::linuxcnc(&job, &pl, &title).map_err(|e| e.to_string())?;
    std::fs::write(out, &program).map_err(|e| format!("{out}: {e}"))?;
    for op in &pl.ops {
        println!("  T{} {:?} {} ({} moves)", op.tool, op.kind, op.name, op.moves.len());
    }
    for w in &pl.warnings {
        println!("warning: {w}");
    }
    let (report, left) = sim::simulate(&job, &pl).map_err(|e| e.to_string())?;
    if let Some(svg) = args.get(2) {
        std::fs::write(svg, sim::svg(&pl, &left, 8.0).map_err(|e| e.to_string())?).map_err(|e| format!("{svg}: {e}"))?;
    }
    println!("{}", serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?);
    println!("{} lines in {out}", program.lines().count());
    Ok(())
}
