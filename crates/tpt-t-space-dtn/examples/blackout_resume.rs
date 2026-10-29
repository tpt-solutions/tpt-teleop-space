//! A bundle is half-transmitted when the link drops. The store keeps the
//! cursor, so transmission resumes exactly where it stopped.
//!
//! Run with: `cargo run -p tpt-t-space-dtn --example blackout_resume`

use tpt_t_space_dtn::bundle::{BundleStore, EndpointId, Priority};
use tpt_t_space_dtn::link::{DtnAgent, LinkError};

fn main() {
    let mut agent = DtnAgent::new(BundleStore::new(16, 1024));
    let telemetry = [0x5Au8; 300];
    agent
        .submit(EndpointId::EARTH_OPS, &telemetry, Priority::Normal, 2, 0)
        .expect("bundle fits the store");

    // A new agent starts with the link down; bring it up for the first flit.
    agent.set_link(true);
    print_flit("link up  ", agent.poll());

    // The link drops mid-bundle: the poll reports it and sends nothing.
    agent.set_link(false);
    match agent.poll() {
        Some(Err(LinkError::LinkDown)) => println!("link down — holding the bundle in NVRAM"),
        other => println!("unexpected: {other:?}"),
    }

    // The link returns; transmission continues from the saved cursor.
    agent.set_link(true);
    let mut final_seen = false;
    while let Some(Ok(egress)) = agent.poll() {
        print_flit("link back", Some(Ok(egress)));
        if egress.final_flit {
            agent.deliver(egress.sector).expect("custody released");
            final_seen = true;
        }
    }
    println!("delivered: {final_seen}, pending: {}", agent.pending());
}

fn print_flit(label: &str, flit: Option<Result<tpt_t_space_dtn::link::Egress, LinkError>>) {
    if let Some(Ok(e)) = flit {
        println!(
            "{label}: bytes {}..{}{}",
            e.from,
            e.to,
            if e.final_flit { " (final)" } else { "" }
        );
    }
}
