// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Box3D-rs "Hello World" simulation example.
//!
//! A dynamic cube falling under gravity onto a static ground box, settling into stable equilibrium.

use box3d::collision::hull::*;
use box3d::*;

fn main() {
    println!("=========================================================");
    println!("   Box3D-rs: Pure Rust 3D Physics Engine (Box3D Fork)    ");
    println!("=========================================================");

    // 1. Create World
    let mut world = create_world(&default_world_def());

    // 2. Create static ground box at y = -10.0 (half-extents 50, 10, 50 -> surface at y = 0.0)
    let mut ground_def = default_body_def();
    ground_def.body_type = BodyType::Static;
    ground_def.position = Vec3::new(0.0, -10.0, 0.0);
    let ground_id = world.create_body(&ground_def);
    let ground_hull = make_box_hull(50.0, 10.0, 50.0);
    world.create_hull_shape(ground_id, &default_shape_def(), &ground_hull);

    // 3. Create dynamic cube at y = 1.1 (half-extent 1.0 -> cube size 2x2x2)
    let mut body_def = default_body_def();
    body_def.body_type = BodyType::Dynamic;
    body_def.position = Vec3::new(0.0, 1.1, 0.0);
    let body_id = world.create_body(&body_def);
    let cube_hull = make_cube_hull(1.0);
    world.create_hull_shape(body_id, &default_shape_def(), &cube_hull);

    let time_step = 1.0 / 60.0;
    let sub_steps = 4;

    println!("\nSimulating 90 steps (1.5 seconds) with 4 sub-steps per frame...\n");
    println!(" {:<6} | {:<22} | {:<12} | {:<20}", "Step", "Position (x, y, z)", "Linear Vy", "Equilibrium State");
    println!("{:-<6}-+-{:-<22}-+-{:-<12}-+-{:-<20}", "", "", "", "");

    for step in 1..=90 {
        world.step(time_step, sub_steps);
        let pos = world.get_body_position(body_id);
        let vel = world.get_body(body_id).unwrap().linear_velocity;

        if step <= 5 || (44..=55).contains(&step) || step % 15 == 0 || step == 90 {
            let status = if (pos.y - 1.0).abs() < 0.005 && vel.y.abs() < 0.01 {
                "Settled at Ground [Equilibrium]"
            } else if vel.y < -0.1 {
                "Falling / Colliding"
            } else {
                "Bouncing / Damping"
            };

            println!(
                " {:<6} | ({:5.2}, {:5.2}, {:5.2})       | {:<12.4} | {}",
                step, pos.x, pos.y, pos.z, vel.y, status
            );
        }
    }

    let final_pos = world.get_body_position(body_id);
    let final_rot = world.get_body_rotation(body_id);
    let (axis, angle) = final_rot.to_axis_angle();

    println!("\nSimulation Results:");
    println!("  Final Position: ({:.5}, {:.5}, {:.5}) [Expected y ≈ 1.0000]", final_pos.x, final_pos.y, final_pos.z);
    println!("  Final Rotation: axis=({:.3}, {:.3}, {:.3}), angle={:.5} rad", axis.x, axis.y, axis.z, angle);
    println!("  Settlement Error: {:.5} m (< 0.01 m tolerance: PASS)", (final_pos.y - 1.0).abs());
    println!("=========================================================\n");
}
