// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

use box3d::collision::hull::*;
use box3d::*;

#[test]
fn test_joint_creation_and_destruction() {
    let mut world = create_world(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.body_type = BodyType::Static;
    let ground_id = world.create_body(&ground_def);

    let mut body_def = default_body_def();
    body_def.body_type = BodyType::Dynamic;
    body_def.position = Vec3::new(0.0, 4.0, 0.0);
    let body_id = world.create_body(&body_def);
    let box_hull = make_cube_hull(0.5);
    world.create_hull_shape(body_id, &default_shape_def(), &box_hull);

    // Distance Joint
    let dist_def = DistanceJointDef {
        body_id_a: ground_id,
        body_id_b: body_id,
        local_anchor_a: Vec3::new(0.0, 4.0, 0.0),
        local_anchor_b: Vec3::ZERO,
        length: 2.0,
        ..Default::default()
    };
    let joint_id = world.create_joint(JointKind::Distance(dist_def));

    // Step world with joint active
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }

    // Destroy joint
    world.destroy_joint(joint_id);

    // Step world again
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }
}

#[test]
fn test_revolute_and_weld_joints() {
    let mut world = create_world(&default_world_def());

    let ground_id = world.create_body(&default_body_def());
    let mut body_def = default_body_def();
    body_def.body_type = BodyType::Dynamic;
    body_def.position = Vec3::new(2.0, 0.0, 0.0);
    let body_id = world.create_body(&body_def);
    let box_hull = make_cube_hull(0.5);
    world.create_hull_shape(body_id, &default_shape_def(), &box_hull);

    // Revolute Joint
    let rev_def = RevoluteJointDef {
        body_id_a: ground_id,
        body_id_b: body_id,
        local_anchor_a: Vec3::ZERO,
        local_anchor_b: Vec3::new(-2.0, 0.0, 0.0),
        local_axis_a: Vec3::Z,
        enable_limit: true,
        lower_angle: -0.5,
        upper_angle: 0.5,
        ..Default::default()
    };
    let rev_id = world.create_joint(JointKind::Revolute(rev_def));
    world.step(1.0 / 60.0, 4);
    world.destroy_joint(rev_id);

    // Weld Joint
    let weld_def = WeldJointDef {
        body_id_a: ground_id,
        body_id_b: body_id,
        local_anchor_a: Vec3::new(2.0, 0.0, 0.0),
        local_anchor_b: Vec3::ZERO,
        reference_rotation: Quat::IDENTITY,
    };
    let weld_id = world.create_joint(JointKind::Weld(weld_def));
    world.step(1.0 / 60.0, 4);
    world.destroy_joint(weld_id);
}
