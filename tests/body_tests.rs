// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

use approx::assert_relative_eq;
use box3d::collision::hull::*;
use box3d::*;

#[test]
fn test_body_creation_and_mass() {
    let mut world = create_world(&default_world_def());

    let mut body_def = default_body_def();
    body_def.body_type = BodyType::Dynamic;
    body_def.position = Vec3::new(1.0, 2.0, 3.0);
    body_def.linear_damping = 0.1;
    body_def.angular_damping = 0.1;

    let body_id = world.create_body(&body_def);
    assert_eq!(world.get_body_position(body_id), Vec3::new(1.0, 2.0, 3.0));

    // Initially body has no shapes, so mass is 0
    let body = world.get_body(body_id).unwrap();
    assert_eq!(body.mass, 0.0);

    // Attach a cube hull shape (half-extent 1.0 -> volume 8.0, density 1.0 -> mass 8.0)
    let box_hull = make_cube_hull(1.0);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let shape_id = world.create_hull_shape(body_id, &shape_def, &box_hull);

    let body_after = world.get_body(body_id).unwrap();
    assert_relative_eq!(body_after.mass, 8.0, epsilon = 1e-4);
    assert_relative_eq!(body_after.inv_mass, 1.0 / 8.0, epsilon = 1e-4);

    // Apply linear impulse
    let impulse = Vec3::new(16.0, 0.0, 0.0);
    world.get_body_mut(body_id).unwrap().apply_linear_impulse(impulse);
    assert_relative_eq!(world.get_body(body_id).unwrap().linear_velocity.x, 2.0, epsilon = 1e-4);

    // Destroy shape
    world.destroy_shape(shape_id);
    assert_relative_eq!(world.get_body(body_id).unwrap().mass, 0.0, epsilon = 1e-4);

    // Destroy body
    world.destroy_body(body_id);
    assert!(world.get_body(body_id).is_none());
}

#[test]
fn test_static_and_kinematic_bodies() {
    let mut world = create_world(&default_world_def());

    let mut static_def = default_body_def();
    static_def.body_type = BodyType::Static;
    static_def.position = Vec3::new(0.0, -10.0, 0.0);
    let static_id = world.create_body(&static_def);
    let static_hull = make_box_hull(10.0, 1.0, 10.0);
    world.create_hull_shape(static_id, &default_shape_def(), &static_hull);

    let static_body = world.get_body(static_id).unwrap();
    assert!(static_body.is_static());
    assert_eq!(static_body.mass, 0.0);
    assert_eq!(static_body.inv_mass, 0.0);

    // Step world: static body should not move under gravity
    world.step(1.0 / 60.0, 4);
    assert_eq!(world.get_body_position(static_id), Vec3::new(0.0, -10.0, 0.0));
}
