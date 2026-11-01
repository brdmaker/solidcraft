//! Plan a job and write its LinuxCNC program: `cargo run -p solidcraft-cam --example gcode --
//! JOB.json OUT.ngc`. The parts' `design`s are STEP files relative to the job.

use std::path::Path;
use std::process::ExitCode;

use solidcraft_cam::job::Job;
use solidcraft_cam::plan::plan_job;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("gcode: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(job_path), Some(out)) = (args.first(), args.get(1)) else {
        return Err("usage: gcode JOB.json OUT.ngc".into());
    };
    let base = Path::new(job_path).parent().unwrap_or(Path::new("."));
    let load = |lib: &str| std::fs::read_to_string(base.join(lib)).map_err(|e| solidcraft_cam::CamError::Invalid(format!("{lib}: {e}")));
    let text = std::fs::read_to_string(job_path).map_err(|e| format!("{job_path}: {e}"))?;
    let job = Job::from_json_with(&text, &load).map_err(|e| e.to_string())?;
    let mut bodies = Vec::new();
    for p in &job.parts {
        let file = base.join(&p.design);
        let text = std::fs::read_to_string(&file).map_err(|e| format!("{}: {e}", file.display()))?;
        let read = solidcraft_kernel::step_import(&text).map_err(|e| format!("{}: {e}", file.display()))?;
        let body = read.bodies.iter().find(|b| b.name == p.body).or(read.bodies.first()).ok_or(format!("{}: no solid", file.display()))?;
        bodies.push(body.body.clone());
    }
    let plan = plan_job(&job, &bodies).map_err(|e| e.to_string())?;
    let title = job.parts.iter().map(|p| p.label()).collect::<Vec<_>>().join(", ");
    let program = solidcraft_cam::post::linuxcnc(&job, &plan.ops, &title).map_err(|e| e.to_string())?;
    std::fs::write(out, &program).map_err(|e| format!("{out}: {e}"))?;
    println!("{} lines in {out}", program.lines().count());
    for w in &plan.warnings {
        println!("warning: {w}");
    }
    Ok(())
}
