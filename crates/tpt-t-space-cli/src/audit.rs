//! `tpts audit [--strict]` — the lock, recursion, panic, and allocation
//! audits plus `cargo deny`, from one command.

use std::process::{Command, ExitCode};

use crate::{CliResult, bash_program, has_flag, repo_root};

/// Audit scripts under `tools/`, run in order.
const SCRIPTS: [(&str, &str); 4] = [
    ("lock-free hot path (hard gate)", "tools/lock-audit.sh"),
    ("recursion (static call graph)", "tools/no-recursion.sh"),
    ("hot-path panics", "tools/no-panic-audit.sh"),
    ("allocation (report)", "tools/alloc-audit.sh"),
];

pub fn run(args: &[String]) -> CliResult {
    if let Some(unknown) = args.iter().find(|a| a.as_str() != "--strict") {
        return Err(format!("audit: unknown argument `{unknown}`"));
    }
    let strict = has_flag(args, "--strict");
    let root = repo_root()?;
    let bash =
        bash_program().ok_or("Git Bash not found; install Git for Windows or set TPT_BASH")?;

    let mut results: Vec<(&str, bool)> = Vec::new();
    for (name, script) in SCRIPTS {
        let mut cmd = Command::new(&bash);
        cmd.arg(script).current_dir(&root);
        if strict {
            cmd.env("TPT_NO_PANIC_STRICT", "1");
        }
        let status = cmd
            .status()
            .map_err(|e| format!("could not start {} for {script}: {e}", bash.display()))?;
        results.push((name, status.success()));
    }
    let deny = Command::new("cargo")
        .args(["deny", "check"])
        .current_dir(&root)
        .status()
        .map_err(|e| format!("could not start cargo deny: {e}"))?;
    results.push(("cargo-deny (MIT chain + §7 bans)", deny.success()));

    println!();
    let mut failed = 0usize;
    for (name, ok) in &results {
        println!("{:<6} {name}", if *ok { "PASS" } else { "FAIL" });
        failed += usize::from(!ok);
    }
    if strict {
        println!("\n(strict: hot-path panic audit runs with TPT_NO_PANIC_STRICT=1)");
    }
    if failed == 0 {
        println!("\nAll audits passed.");
        Ok(ExitCode::SUCCESS)
    } else {
        println!("\n{failed} audit(s) failed.");
        Ok(ExitCode::FAILURE)
    }
}
