//! `tpts budgets` — the three strict benches against the spec budgets.
//!
//! Each bench is run with its `TPT_*_STRICT=1` switch, so a budget miss
//! fails the bench. Benches run from the crate directory: rustup picks the
//! toolchain from the working directory, so `--manifest-path` from the repo
//! root would ignore the satellite crate's nightly `rust-toolchain.toml`.
//! `RUSTUP_TOOLCHAIN` is also cleared, since `cargo run` sets it and it would
//! override that file.

use std::process::{Command, ExitCode};

use crate::{CliResult, repo_root};

struct Budget {
    name: &'static str,
    spec: &'static str,
    dir: &'static str,
    strict_env: &'static str,
}

const BUDGETS: [Budget; 3] = [
    Budget {
        name: "TMR vote-and-reset-trigger",
        spec: "< 50 µs",
        dir: "crates/tpt-t-space-radiation",
        strict_env: "TPT_RAD_STRICT",
    },
    Budget {
        name: "ADCS control loop",
        spec: "< 200 µs",
        dir: "crates/tpt-t-space-satellite",
        strict_env: "TPT_ADCS_STRICT",
    },
    Budget {
        name: "Rover replan (15 m detour)",
        spec: "< 50 ms (§6: 40 ms)",
        dir: "crates/tpt-t-space-predictive",
        strict_env: "TPT_PRED_STRICT",
    },
];

pub fn run() -> CliResult {
    let root = repo_root()?;
    println!("{:<30} {:<22} {:<6}", "budget", "spec", "result");
    let mut failed = 0usize;
    for b in &BUDGETS {
        let out = Command::new("cargo")
            .arg("bench")
            // `cargo run` exports RUSTUP_TOOLCHAIN, which would override the
            // crate-local `rust-toolchain.toml` pin; clear it for the child.
            .env_remove("RUSTUP_TOOLCHAIN")
            .env(b.strict_env, "1")
            .current_dir(root.join(b.dir))
            .output()
            .map_err(|e| format!("could not start cargo bench: {e}"))?;
        let ok = out.status.success();
        println!(
            "{:<30} {:<22} {}",
            b.name,
            b.spec,
            if ok { "PASS" } else { "FAIL" }
        );
        if !ok {
            failed += 1;
            let stderr = String::from_utf8_lossy(&out.stderr);
            for line in stderr
                .lines()
                .rev()
                .take(5)
                .collect::<Vec<_>>()
                .iter()
                .rev()
            {
                println!("    {line}");
            }
        }
    }
    if failed == 0 {
        Ok(ExitCode::SUCCESS)
    } else {
        println!("\n{failed} budget(s) missed.");
        Ok(ExitCode::FAILURE)
    }
}
