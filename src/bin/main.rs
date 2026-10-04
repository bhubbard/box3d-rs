// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Box3D-rs command line interface and benchmark runner.

use box3d::collision::hull::*;
use box3d::*;
use clap::{Parser, Subcommand};
use colored::*;
use std::time::Instant;

#[derive(Parser, Debug)]
#[command(name = "box3d-cli")]
#[command(author = "Brandon Hubbard & Erin Catto")]
#[command(version = "0.1.0")]
#[command(about = "High-performance pure Rust 3D physics engine forked from Erin Catto's Box3D", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Run an interactive physics simulation demo
    Demo {
        /// Demo type: 'hello', 'stack', or 'restitution'
        #[arg(short, long, default_value = "hello")]
        scenario: String,
        /// Number of frames to simulate
        #[arg(short, long, default_value_t = 90)]
        frames: u32,
    },
    /// Benchmark solver performance with stacked bodies
    Bench {
        /// Number of stacked bodies
        #[arg(short, long, default_value_t = 50)]
        bodies: usize,
        /// Number of simulation steps
        #[arg(short, long, default_value_t = 500)]
        steps: usize,
    },
    /// Print engine architecture and feature info
    Info,
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Demo { scenario, frames } => run_demo(&scenario, frames),
        Commands::Bench { bodies, steps } => run_bench(bodies, steps),
        Commands::Info => print_info(),
    }
}

fn print_info() {
    println!("{}", "=========================================================".cyan());
    println!("{}", "   Box3D-rs: Pure Rust 3D Physics Engine Architecture    ".bold().yellow());
    println!("{}", "=========================================================".cyan());
    println!("  Version:            0.1.0 (Fork of Box3D by Erin Catto)");
    println!("  Integrator:         Soft Step Constraint Sub-stepping");
    println!("  Collision Detection: SAT (Separating Axis Theorem) + Sutherland-Hodgman Polygon Clipping");
    println!("  Broad-phase:        Dual Dynamic AABB Trees (Static & Dynamic)");
    println!("  Constraint Solver:  Soft compliance, 2-axis Coulomb friction, restitution");
    println!("  Primitives:         Sphere, Capsule, Polyhedral Convex Hull, Compound");
    println!("  Joints:             Distance, Revolute, Prismatic, Spherical, Weld");
    println!("  Safety:             100% Safe Pure Rust Core (Zero C bindings)");
    println!("{}", "=========================================================\n".cyan());
}

fn run_demo(scenario: &str, frames: u32) {
    println!("{}", format!("Running Demo: Scenario '{}' ({} frames)", scenario, frames).bold().green());

    let mut world = create_world(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.body_type = BodyType::Static;
    ground_def.position = Vec3::new(0.0, -10.0, 0.0);
    let ground_id = world.create_body(&ground_def);
    let ground_hull = make_box_hull(50.0, 10.0, 50.0);
    world.create_hull_shape(ground_id, &default_shape_def(), &ground_hull);

    let mut body_def = default_body_def();
    body_def.body_type = BodyType::Dynamic;
    body_def.position = Vec3::new(0.0, 1.1, 0.0);
    let body_id = world.create_body(&body_def);
    let cube_hull = make_cube_hull(1.0);
    world.create_hull_shape(body_id, &default_shape_def(), &cube_hull);

    let dt = 1.0 / 60.0;
    for step in 1..=frames {
        world.step(dt, 4);
        let pos = world.get_body_position(body_id);
        let vel = world.get_body(body_id).unwrap().linear_velocity;

        if step % 15 == 0 || step == frames {
            println!(
                "  Frame {:3}: Position=({:6.3}, {:6.3}, {:6.3}), Vy={:6.3} m/s",
                step, pos.x, pos.y, pos.z, vel.y
            );
        }
    }

    let final_pos = world.get_body_position(body_id);
    println!("\n  Final settlement: y = {:.5} m (error: {:.5} m)", final_pos.y, (final_pos.y - 1.0).abs());
}

fn run_bench(bodies_count: usize, steps: usize) {
    println!("{}", format!("Benchmarking Box3D-rs: {} bodies across {} steps...", bodies_count, steps).bold().yellow());

    let mut world = create_world(&default_world_def());

    // Ground
    let mut ground_def = default_body_def();
    ground_def.body_type = BodyType::Static;
    ground_def.position = Vec3::new(0.0, -1.0, 0.0);
    let ground_id = world.create_body(&ground_def);
    let ground_hull = make_box_hull(100.0, 1.0, 100.0);
    world.create_hull_shape(ground_id, &default_shape_def(), &ground_hull);

    // Stacks
    let box_hull = make_cube_hull(0.5);
    for i in 0..bodies_count {
        let mut def = default_body_def();
        def.body_type = BodyType::Dynamic;
        let col = i % 5;
        let row = (i / 5) % 5;
        let height = i / 25;
        def.position = Vec3::new(col as f32 * 1.2 - 2.4, 0.5 + height as f32 * 1.05, row as f32 * 1.2 - 2.4);
        let id = world.create_body(&def);
        world.create_hull_shape(id, &default_shape_def(), &box_hull);
    }

    let start = Instant::now();
    let dt = 1.0 / 60.0;
    for _ in 0..steps {
        world.step(dt, 4);
    }
    let elapsed = start.elapsed();

    let ms_per_step = elapsed.as_secs_f64() * 1000.0 / steps as f64;
    let fps_equivalent = 1000.0 / ms_per_step;

    println!("\n{}", "Benchmark Results:".bold().green());
    println!("  Total Elapsed:      {:.3} ms", elapsed.as_secs_f64() * 1000.0);
    println!("  Time per Step:      {:.3} ms (at 4 sub-steps)", ms_per_step);
    println!("  Equivalent Rate:    {:.1} simulated FPS", fps_equivalent);
    println!("  Real-time speedup:  {:.1}x realtime at 60Hz", fps_equivalent / 60.0);
}
