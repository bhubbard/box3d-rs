// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Integration test verifying Erin Catto's authentic Box3D HelloWorld simulation:
//! A unit cube dynamic body drops from y = 4.0 onto a static ground slab at y = -10.0
//! with half-height 10.0 (top surface at y = 0.0). Over 90 steps at 60 Hz with 4 sub-steps,
//! the cube comes to rest at y ≈ 1.00 ± 0.01.

use box3d::*;

#[test]
fn test_hello_world() {
    // 1. Construct world object with gravity (0, -10, 0)
    let mut world_def = default_world_def();
    world_def.gravity = Vec3::new(0.0, -10.0, 0.0);

    let mut world = create_world(&world_def);
    assert!(world.is_valid());

    // 2. Define static ground body
    let mut ground_body_def = default_body_def();
    ground_body_def.position = Vec3::new(0.0, -10.0, 0.0);

    let ground_id = world.create_body(&ground_body_def);
    assert!(ground_id.is_valid());

    // 3. Define ground box shape: 50m x 10m x 50m half-widths
    let ground_box = make_box_hull(50.0, 10.0, 50.0);
    let ground_shape_def = default_shape_def();
    world.create_hull_shape(ground_id, &ground_shape_def, &ground_box);

    // 4. Define dynamic body at y = 4.0
    let mut body_def = default_body_def();
    body_def.body_type = BodyType::Dynamic;
    body_def.position = Vec3::new(0.0, 4.0, 0.0);

    let body_id = world.create_body(&body_def);
    assert!(body_id.is_valid());

    // 5. Define dynamic unit cube shape (half-extent 1.0m on each axis)
    let dynamic_box = make_cube_hull(1.0);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.material.friction = 0.3;

    world.create_hull_shape(body_id, &shape_def, &dynamic_box);

    // 6. Simulate 90 steps at 60 Hz with 4 sub-steps
    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;

    let mut final_pos = world.get_body_position(body_id);

    for step in 0..90 {
        world.step(time_step, sub_step_count);
        final_pos = world.get_body_position(body_id);
        let rot = world.get_body_rotation(body_id);

        if (44..=65).contains(&step) {
            println!(
                "Step {:2}: pos = ({:6.4}, {:6.4}, {:6.4}), vy = {:6.4}",
                step + 1,
                final_pos.x,
                final_pos.y,
                final_pos.z,
                world.get_body(body_id).unwrap().linear_velocity.y
            );
        } else if step % 15 == 0 || step == 89 {
            println!(
                "Step {:2}: pos = ({:4.2}, {:4.2}, {:4.2}), rot = ({:4.2}, {:4.2}, {:4.2}, {:4.2})",
                step + 1,
                final_pos.x,
                final_pos.y,
                final_pos.z,
                rot.v.x,
                rot.v.y,
                rot.v.z,
                rot.s
            );
        }
    }

    // 7. Verify settlement position at y = 1.00 ± 0.01 (cube half-height = 1.0 sitting on ground at y = 0.0)
    let final_rot = world.get_body_rotation(body_id);
    let diff_y = (final_pos.y - 1.00).abs();
    println!("Final position: ({:.4}, {:.4}, {:.4})", final_pos.x, final_pos.y, final_pos.z);
    println!("Final rotation: ({:.4}, {:.4}, {:.4}, {:.4})", final_rot.v.x, final_rot.v.y, final_rot.v.z, final_rot.s);
    println!("Difference from y = 1.00: {:.5}", diff_y);

    assert!(
        diff_y < 0.01,
        "Body did not settle at y = 1.00 ± 0.01; actual y = {}",
        final_pos.y
    );
    assert!(final_rot.v.x.abs() < 0.01, "Rotation rx error: {}", final_rot.v.x);
    assert!(final_rot.v.z.abs() < 0.01, "Rotation rz error: {}", final_rot.v.z);
}
