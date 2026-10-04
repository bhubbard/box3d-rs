// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

use box3d::collision::hull::*;
use box3d::collision::shape::Sphere;
use box3d::*;

#[test]
fn test_vertical_box_stack() {
    let mut world = create_world(&default_world_def());

    // Static ground
    let mut ground_def = default_body_def();
    ground_def.body_type = BodyType::Static;
    ground_def.position = Vec3::new(0.0, -1.0, 0.0);
    let ground_id = world.create_body(&ground_def);
    let ground_hull = make_box_hull(20.0, 1.0, 20.0);
    world.create_hull_shape(ground_id, &default_shape_def(), &ground_hull);

    // Stack 3 cubes (half-extent 0.5 -> height 1.0 each)
    let box_hull = make_cube_hull(0.5);
    let mut body_ids = Vec::new();

    for i in 0..3 {
        let mut body_def = default_body_def();
        body_def.body_type = BodyType::Dynamic;
        body_def.position = Vec3::new(0.0, 0.5 + i as f32 * 1.05, 0.0);
        let id = world.create_body(&body_def);
        world.create_hull_shape(id, &default_shape_def(), &box_hull);
        body_ids.push(id);
    }

    // Step simulation for 90 frames
    for s in 0..90 {
        world.step(1.0 / 60.0, 4);
        if s % 10 == 0 || s == 89 {
            println!(
                "Step {:2}: y0={:.3}, y1={:.3}, y2={:.3}",
                s,
                world.get_body_position(body_ids[0]).y,
                world.get_body_position(body_ids[1]).y,
                world.get_body_position(body_ids[2]).y,
            );
        }
    }

    // Verify all 3 boxes are resting stacked without flying away
    let y0 = world.get_body_position(body_ids[0]).y;
    let y1 = world.get_body_position(body_ids[1]).y;
    let y2 = world.get_body_position(body_ids[2]).y;

    assert!(y0 > 0.4 && y0 < 0.6, "Bottom box should rest at y ~ 0.5, got {}", y0);
    assert!(y1 > 1.4 && y1 < 1.6, "Middle box should rest at y ~ 1.5, got {}", y1);
    assert!(y2 > 2.4 && y2 < 2.6, "Top box should rest at y ~ 2.5, got {}", y2);
}

#[test]
fn test_sphere_bounce_restitution() {
    let mut world = create_world(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.body_type = BodyType::Static;
    ground_def.position = Vec3::new(0.0, -1.0, 0.0);
    let ground_id = world.create_body(&ground_def);
    let ground_hull = make_box_hull(20.0, 1.0, 20.0);
    world.create_hull_shape(ground_id, &default_shape_def(), &ground_hull);

    // Dropping sphere 1: zero restitution
    let mut def1 = default_body_def();
    def1.body_type = BodyType::Dynamic;
    def1.position = Vec3::new(-3.0, 4.0, 0.0);
    let id1 = world.create_body(&def1);
    let mut shape_def1 = default_shape_def();
    shape_def1.material.restitution = 0.0;
    world.create_sphere_shape(id1, &shape_def1, Sphere { radius: 0.5 });

    // Step until impact and settlement
    for _ in 0..100 {
        world.step(1.0 / 60.0, 4);
    }

    let pos1 = world.get_body_position(id1);
    assert!((pos1.y - 0.5).abs() < 0.05, "Zero restitution sphere should settle at y ~ 0.5, got {}", pos1.y);
}
