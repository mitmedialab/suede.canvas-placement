//! Replay a request from JSON on stdin; print the result as JSON.
//!
//!   placement place          < request.json
//!   placement free-fractions < request.json
//!   placement occupancy      < request.json

use canvas_placement::{
    free_fractions, occupancy, place, FreeFractionsRequest, OccupancyRequest, PlaceRequest,
};
use std::io::Read;

fn main() {
    let op = std::env::args().nth(1).unwrap_or_default();
    let mut input = String::new();
    std::io::stdin().read_to_string(&mut input).expect("read stdin");
    let out = match op.as_str() {
        "place" => {
            let req: PlaceRequest = serde_json::from_str(&input).unwrap_or_else(|e| fail(&e));
            serde_json::to_string_pretty(&place(&req).unwrap_or_else(|e| fail(&e)))
        }
        "free-fractions" => {
            let req: FreeFractionsRequest = serde_json::from_str(&input).unwrap_or_else(|e| fail(&e));
            serde_json::to_string_pretty(&free_fractions(&req))
        }
        "occupancy" => {
            let req: OccupancyRequest = serde_json::from_str(&input).unwrap_or_else(|e| fail(&e));
            serde_json::to_string(&occupancy(&req).unwrap_or_else(|e| fail(&e)))
        }
        _ => fail(&"usage: placement <place|free-fractions|occupancy> < request.json"),
    };
    println!("{}", out.expect("serialize"));
}

fn fail(e: &dyn std::fmt::Display) -> ! {
    eprintln!("error: {e}");
    std::process::exit(1)
}
