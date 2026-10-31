//! Plans a job whose parts are STEP files and writes its toolpath preview (PNG or SVG), without
//! the rest of SolidCraft: `cargo run -p solidcraft-cam --example preview -- JOB.json OUT.png [WIDTH]`.
//! Part designs are relative to the job file. Prints the operations and any warnings.

use std::path::Path;
use std::process::ExitCode;

use solidcraft_cam::job::Job;
use solidcraft_cam::plan::plan_job;
use solidcraft_cam::preview::{Scene, png, svg};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("preview: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(job_path), Some(out)) = (args.first(), args.get(1)) else {
        return Err("usage: preview JOB.json OUT.png|OUT.svg [WIDTH]".into());
    };
    let width: u32 = args.get(2).and_then(|w| w.parse().ok()).unwrap_or(1600);
    let job = Job::from_json(&std::fs::read_to_string(job_path).map_err(|e| format!("{job_path}: {e}"))?).map_err(|e| e.to_string())?;
    let base = Path::new(job_path).parent().unwrap_or(Path::new("."));
    let mut bodies = Vec::new();
    for p in &job.parts {
        let file = base.join(&p.design);
        let text = std::fs::read_to_string(&file).map_err(|e| format!("{}: {e}", file.display()))?;
        let read = solidcraft_kernel::step_import(&text).map_err(|e| format!("{}: {e}", file.display()))?;
        let body = read.bodies.iter().find(|b| b.name == p.body).or(read.bodies.first()).ok_or(format!("{}: no solid", file.display()))?;
        bodies.push(body.body.clone());
    }
    let plan = plan_job(&job, &bodies).map_err(|e| e.to_string())?;
    let outlines = plan.placed_outlines();
    let scene = Scene { sheet: job.stock.size, margin: job.nest.margin, outlines: &outlines, ops: &plan.ops, clearance: job.strategy.clearance };
    if out.ends_with(".svg") {
        std::fs::write(out, svg(&scene)).map_err(|e| format!("{out}: {e}"))?;
    } else {
        std::fs::write(out, png(&scene, width).map_err(|e| e.to_string())?).map_err(|e| format!("{out}: {e}"))?;
    }
    println!("{} copies placed, {} operations:", plan.nesting.placements.len(), plan.ops.len());
    for op in &plan.ops {
        println!("  T{:<3} {:?} {} ({} moves)", op.tool, op.kind, op.name, op.moves.len());
    }
    for w in &plan.warnings {
        println!("warning: {w}");
    }
    Ok(())
}
