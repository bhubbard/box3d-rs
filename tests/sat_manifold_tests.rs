// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

use approx::assert_relative_eq;
use box3d::collision::hull::*;
use box3d::collision::manifold::*;
use box3d::collision::sat::*;
use box3d::collision::shape::{Capsule, Sphere};
use box3d::math::*;

#[test]
fn test_sphere_sphere_collision() {
    let s_a = Sphere { radius: 1.0 };
    let s_b = Sphere { radius: 1.0 };

    // Separated by 0.5 units along X
    let xf_a = Transform::from_translation(Vec3::new(0.0, 0.0, 0.0));
    let xf_b_sep = Transform::from_translation(Vec3::new(2.5, 0.0, 0.0));
    assert!(collide_spheres(&s_a, xf_a, &s_b, xf_b_sep).is_none());

    // Overlapping by 0.2 units along X (distance = 1.8)
    let xf_b_overlap = Transform::from_translation(Vec3::new(1.8, 0.0, 0.0));
    let manifold = collide_spheres(&s_a, xf_a, &s_b, xf_b_overlap).expect("Should collide");

    assert_relative_eq!(manifold.normal.x, 1.0, epsilon = 1e-5);
    assert_relative_eq!(manifold.normal.y, 0.0, epsilon = 1e-5);
    assert_relative_eq!(manifold.normal.z, 0.0, epsilon = 1e-5);
    assert_eq!(manifold.points.len(), 1);
    assert_relative_eq!(manifold.points[0].separation, -0.2, epsilon = 1e-5);
}

#[test]
fn test_capsule_sphere_collision() {
    let cap = Capsule {
        point1: Vec3::new(-1.0, 0.0, 0.0),
        point2: Vec3::new(1.0, 0.0, 0.0),
        radius: 0.5,
    };
    let sphere = Sphere { radius: 0.5 };

    let xf_cap = Transform::IDENTITY;
    // Sphere at (0.0, 0.8, 0.0): closest point on capsule axis is (0,0,0). Distance = 0.8. Radii sum = 1.0. Overlap = -0.2.
    let xf_sphere = Transform::from_translation(Vec3::new(0.0, 0.8, 0.0));
    let manifold = collide_capsule_and_sphere(&cap, xf_cap, &sphere, xf_sphere).expect("Should collide");

    assert_relative_eq!(manifold.normal.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(manifold.normal.y, 1.0, epsilon = 1e-5);
    assert_relative_eq!(manifold.normal.z, 0.0, epsilon = 1e-5);
    assert_relative_eq!(manifold.points[0].separation, -0.2, epsilon = 1e-5);
}

#[test]
fn test_hull_sat_and_manifold_face_contact() {
    let box_a = make_cube_hull(1.0); // [-1, 1]^3
    let box_b = make_cube_hull(1.0); // [-1, 1]^3

    let xf_a = Transform::from_translation(Vec3::new(0.0, 0.0, 0.0));
    // Separated along Y: box A top is at y=1.0, box B bottom is at y=1.5 (center at y=2.5)
    let xf_b_sep = Transform::from_translation(Vec3::new(0.0, 2.5, 0.0));
    let xf_rel_sep = xf_a.inv_mul_transform(xf_b_sep);
    let axis_sep = find_separating_axis(&box_a, &box_b, xf_rel_sep, 1.0).unwrap();
    assert_relative_eq!(axis_sep.separation, 0.5, epsilon = 1e-4);

    // Overlapping along Y: box B center at y=1.9 -> penetration = -0.1
    let xf_b_overlap = Transform::from_translation(Vec3::new(0.0, 1.9, 0.0));
    let xf_rel_overlap = xf_a.inv_mul_transform(xf_b_overlap);
    let axis_overlap = find_separating_axis(&box_a, &box_b, xf_rel_overlap, 1.0).unwrap();
    assert_relative_eq!(axis_overlap.separation, -0.1, epsilon = 1e-4);

    // Manifold generation: two identical square faces touching produce 4 clipped contact points
    let manifold = collide_hulls(&box_a, xf_a, &box_b, xf_b_overlap).expect("Manifold should exist");
    assert_eq!(manifold.points.len(), 4);
    assert_relative_eq!(manifold.normal.y.abs(), 1.0, epsilon = 1e-4);
    for pt in &manifold.points {
        assert_relative_eq!(pt.separation, -0.1, epsilon = 1e-3);
    }
}
