//! Plan a job, simulate cutting it on a height map and compare the result with its parts:
//! `cargo run -p solidcraft-cam --example simulate -- JOB.json OUT.png [CELL_MM]`. Prints the
//! report as JSON; the PNG shows the machined sheet with the spots the report names.

use std::path::Path;
use std::process::ExitCode;

use solidcraft_cam::job::Job;
use solidcraft_cam::plan::plan_job;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("simulate: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(job_path), Some(out)) = (args.first(), args.get(1)) else {
        return Err("usage: simulate JOB.json OUT.png [CELL_MM]".into());
    };
    let cell: f64 = args.get(2).and_then(|c| c.parse().ok()).unwrap_or(0.25);
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
    let (report, map) = solidcraft_cam::sim::simulate(&job, &bodies, &plan, cell).map_err(|e| e.to_string())?;
    let png = solidcraft_cam::sim::png(&map, job.stock.thickness, Some(&report), 2000).map_err(|e| e.to_string())?;
    std::fs::write(out, png).map_err(|e| format!("{out}: {e}"))?;
    println!("{}", serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?);
    Ok(())
}
