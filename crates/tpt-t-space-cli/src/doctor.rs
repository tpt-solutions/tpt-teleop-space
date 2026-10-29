//! `tpts doctor` — check the toolchain the workspace needs and print the
//! exact command for each gap.

use std::process::{Command, ExitCode};

use crate::{CliResult, bash_program};

/// One environment check.
struct Check {
    name: &'static str,
    /// The probed version string, or `None` when the tool is missing.
    found: Option<String>,
    /// Command to run when `found` is `None`.
    fix: &'static str,
}

/// Run `program args…`; its trimmed stdout on success.
fn probe(program: &str, args: &[&str]) -> Option<String> {
    let out = Command::new(program).args(args).output().ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_owned())
}

/// The fix for bash differs by host: Git Bash on Windows, a package
/// manager elsewhere.
fn bash_fix() -> &'static str {
    if cfg!(windows) {
        "install Git for Windows (it ships Git Bash), or point TPT_BASH at its bash.exe"
    } else {
        "install bash through your package manager"
    }
}

pub fn run() -> CliResult {
    let installed_targets = probe("rustup", &["target", "list", "--installed"]).unwrap_or_default();
    let toolchains = probe("rustup", &["toolchain", "list"]).unwrap_or_default();
    let checks = [
        Check {
            name: "rustc",
            found: probe("rustc", &["--version"]),
            fix: "install Rust from https://rustup.rs",
        },
        Check {
            name: "cargo",
            found: probe("cargo", &["--version"]),
            fix: "install Rust from https://rustup.rs",
        },
        Check {
            name: "nightly toolchain (satellite, rover)",
            found: toolchains
                .lines()
                .find(|l| l.starts_with("nightly"))
                .map(str::to_owned),
            fix: "rustup toolchain install nightly --component rustfmt --component clippy",
        },
        Check {
            name: "aarch64-unknown-none-softfloat target (edge)",
            found: installed_targets
                .lines()
                .any(|l| l.trim() == "aarch64-unknown-none-softfloat")
                .then(|| "installed".to_owned()),
            fix: "rustup target add aarch64-unknown-none-softfloat",
        },
        Check {
            name: "cargo-deny",
            found: probe("cargo", &["deny", "--version"]),
            fix: "cargo install cargo-deny --locked",
        },
        Check {
            name: "bash (audit scripts)",
            found: bash_program().and_then(|bash| {
                let version = probe(&bash.to_string_lossy(), &["--version"])?;
                Some(format!("{} ({})", version.lines().next()?, bash.display()))
            }),
            fix: bash_fix(),
        },
    ];

    let mut missing = 0usize;
    for check in &checks {
        match &check.found {
            Some(version) => println!("[ok]      {:<46} {version}", check.name),
            None => {
                missing += 1;
                println!("[missing] {:<46}", check.name);
                println!("          fix: {}", check.fix);
            }
        }
    }
    if missing == 0 {
        println!("\nAll checks passed.");
        Ok(ExitCode::SUCCESS)
    } else {
        println!("\n{missing} check(s) need attention.");
        Ok(ExitCode::FAILURE)
    }
}
