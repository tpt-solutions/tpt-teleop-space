//! `tpts` — the adoption CLI for tpt-teleop-space.
//!
//! One binary for the chores around the workspace: check the toolchain,
//! run the audits and strict budgets without hand-running `.sh` files,
//! decode CCSDS streams, write replay files, and scaffold a mission crate.
//! Argument parsing is hand-rolled to keep the zero-dependency policy.

mod audit;
mod blackbox_cmd;
mod budgets;
mod campaign_cmd;
mod ccsds;
mod doctor;
mod new;
mod seq;
mod sim_cmds;

use std::path::PathBuf;
use std::process::ExitCode;

const USAGE: &str = "\
tpts — tpt-teleop-space adoption CLI

USAGE:
    tpts <COMMAND> [ARGS]

COMMANDS:
    doctor                                  check rustc, nightly, cargo-deny, the
                                            aarch64 target and bash; print each fix
    audit [--strict]                        lock, recursion, panic and alloc audits
                                            plus cargo-deny (--strict: no-panic gate)
    budgets                                 run the three strict benches against
                                            the spec budgets (pass/fail table)
    ccsds <hex|file> [--crc]                decode a CCSDS Space Packet stream
    orbit [--orbits N] [--step S] [--out F] propagate a LEO survey, write a replay
    profile [NAME]                          list mission-profile presets, or show one
    scenario mars [--out F]                 run the spec §6 Mars Sample Collection
                                            flow, print its timeline, write a replay
    blackbox <hex|file>                     decode a Safe Mode black-box image
    campaign seu|blackout [--seeds N]       seeded fault-injection sweep (SEU rate, blackout length)
    seq <steps>                             lint an uplink sequence and dry-run it
                                            over the Mars terrain, e.g. 'drive 1; turn 90'
    new <rover|satellite|orbital> [--name N] scaffold a mission crate wired to core
                                            Safe Mode and DTN, with a smoke test
    help                                    print this message
";

/// Outcome of a command: a process exit code, or a usage/setup error.
pub type CliResult = Result<ExitCode, String>;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let rest = args.get(1..).unwrap_or_default();
    let result = match args.first().map(String::as_str) {
        Some("doctor") => doctor::run(),
        Some("audit") => audit::run(rest),
        Some("budgets") => budgets::run(),
        Some("ccsds") => ccsds::run(rest),
        Some("orbit") => sim_cmds::orbit(rest),
        Some("profile") => sim_cmds::profile(rest),
        Some("blackbox") => blackbox_cmd::run(rest),
        Some("seq") => seq::run(rest),
        Some("campaign") => campaign_cmd::run(rest),
        Some("scenario") => sim_cmds::scenario(rest),
        Some("new") => new::run(rest),
        None | Some("help" | "--help" | "-h") => {
            print!("{USAGE}");
            Ok(ExitCode::SUCCESS)
        }
        Some(other) => Err(format!("unknown command `{other}`")),
    };
    match result {
        Ok(code) => code,
        Err(msg) => {
            eprintln!("tpts: {msg}\n");
            eprint!("{USAGE}");
            ExitCode::from(2)
        }
    }
}

/// The value following `name` (`--name value`), if present.
pub fn flag_value<'a>(args: &'a [String], name: &str) -> Result<Option<&'a str>, String> {
    match args.iter().position(|a| a == name) {
        None => Ok(None),
        Some(i) => args
            .get(i + 1)
            .map(|v| Some(v.as_str()))
            .ok_or_else(|| format!("{name} needs a value")),
    }
}

/// Whether the bare flag `name` is present.
pub fn has_flag(args: &[String], name: &str) -> bool {
    args.iter().any(|a| a == name)
}

/// The first argument that is neither a flag nor a flag's value.
pub fn positional<'a>(args: &'a [String], value_flags: &[&str]) -> Option<&'a str> {
    let mut skip_next = false;
    for a in args {
        if skip_next {
            skip_next = false;
        } else if value_flags.contains(&a.as_str()) {
            skip_next = true;
        } else if !a.starts_with("--") {
            return Some(a.as_str());
        }
    }
    None
}

/// The bash used to run the audit scripts.
///
/// On Windows a bare `bash` resolves to `C:\Windows\System32\bash.exe`
/// (the WSL launcher), which can block when no distro is installed. Git
/// Bash is therefore located by absolute path; `TPT_BASH` overrides it.
pub fn bash_program() -> Option<PathBuf> {
    if let Some(p) = std::env::var_os("TPT_BASH") {
        let p = PathBuf::from(p);
        return p.is_file().then_some(p);
    }
    if !cfg!(windows) {
        return Some(PathBuf::from("bash"));
    }
    ["ProgramFiles", "ProgramW6432", "ProgramFiles(x86)"]
        .into_iter()
        .filter_map(std::env::var_os)
        .map(|dir| PathBuf::from(dir).join("Git").join("bin").join("bash.exe"))
        .find(|p| p.is_file())
}

/// The repository root: `TPT_ROOT` if set, else the nearest ancestor of
/// the working directory that holds both `Cargo.toml` and `tools/`.
pub fn repo_root() -> Result<PathBuf, String> {
    if let Some(root) = std::env::var_os("TPT_ROOT") {
        return Ok(PathBuf::from(root));
    }
    let start = std::env::current_dir().map_err(|e| e.to_string())?;
    start
        .ancestors()
        .find(|dir| dir.join("Cargo.toml").is_file() && dir.join("tools").is_dir())
        .map(PathBuf::from)
        .ok_or_else(|| "not inside the tpt-teleop-space repository (set TPT_ROOT)".to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(v: &[&str]) -> Vec<String> {
        v.iter().map(|s| (*s).to_owned()).collect()
    }

    #[test]
    fn flag_value_reads_the_following_argument() {
        let a = args(&["--out", "x.replay", "--orbits", "3"]);
        assert_eq!(flag_value(&a, "--out"), Ok(Some("x.replay")));
        assert_eq!(flag_value(&a, "--step"), Ok(None));
        assert!(flag_value(&args(&["--out"]), "--out").is_err());
    }

    #[test]
    fn positional_skips_flags_and_their_values() {
        let a = args(&["--out", "f.replay", "mars"]);
        assert_eq!(positional(&a, &["--out"]), Some("mars"));
        assert_eq!(positional(&args(&["--crc"]), &[]), None);
    }

    #[test]
    fn has_flag_matches_exactly() {
        assert!(has_flag(&args(&["--strict"]), "--strict"));
        assert!(!has_flag(&args(&["--strictly"]), "--strict"));
    }
}
