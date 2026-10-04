// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

use approx::assert_relative_eq;
use box3d::math::*;
use std::f32::consts::PI;

#[test]
fn test_vec3_basic_ops() {
    let zero = Vec3::ZERO;
    let one = Vec3::ONE;
    let two = Vec3::new(2.0, 2.0, 2.0);

    let v = one + two;
    assert_eq!(v, Vec3::new(3.0, 3.0, 3.0));

    let v = zero - two;
    assert_eq!(v, Vec3::new(-2.0, -2.0, -2.0));

    let v = -two;
    assert_eq!(v, Vec3::new(-2.0, -2.0, -2.0));

    let dot = Vec3::new(1.0, 2.0, 3.0).dot(Vec3::new(4.0, -5.0, 6.0));
    assert_eq!(dot, 1.0 * 4.0 + 2.0 * -5.0 + 3.0 * 6.0); // 4 - 10 + 18 = 12

    let cross = Vec3::X.cross(Vec3::Y);
    assert_relative_eq!(cross.x, Vec3::Z.x, epsilon = 1e-6);
    assert_relative_eq!(cross.y, Vec3::Z.y, epsilon = 1e-6);
    assert_relative_eq!(cross.z, Vec3::Z.z, epsilon = 1e-6);

    let perp = Vec3::new(0.50405544, 0.62154805, 0.59967154).perp();
    assert_relative_eq!(perp.dot(Vec3::new(0.50405544, 0.62154805, 0.59967154)), 0.0, epsilon = 1e-6);
}

#[test]
fn test_quat_operations() {
    let q_id = Quat::IDENTITY;
    let axis = Vec3::new(-0.75, 0.5, 1.0).normalize();
    let q = Quat::from_axis_angle(axis, PI);

    // Rotating a vector twice by 180 deg brings it back
    let v = Vec3::new(1.0, 2.0, 3.0);
    let v_rot = q.rotate_vec3(v);
    let v_back = q.rotate_vec3(v_rot);
    assert_relative_eq!(v_back.x, v.x, epsilon = 1e-5);
    assert_relative_eq!(v_back.y, v.y, epsilon = 1e-5);
    assert_relative_eq!(v_back.z, v.z, epsilon = 1e-5);

    // Inverse quat
    let q_inv = q.conjugate();
    let v_inv = q_inv.rotate_vec3(v_rot);
    assert_relative_eq!(v_inv.x, v.x, epsilon = 1e-5);
    assert_relative_eq!(v_inv.y, v.y, epsilon = 1e-5);
    assert_relative_eq!(v_inv.z, v.z, epsilon = 1e-5);

    // Multiplication with identity
    let q_mul = q * q_id;
    assert_relative_eq!(q_mul.v.x, q.v.x, epsilon = 1e-6);
    assert_relative_eq!(q_mul.v.y, q.v.y, epsilon = 1e-6);
    assert_relative_eq!(q_mul.v.z, q.v.z, epsilon = 1e-6);
    assert_relative_eq!(q_mul.s, q.s, epsilon = 1e-6);
}

#[test]
fn test_matrix33_ops() {
    let m = Mat33::from_cols(
        Vec3::new(3.0, 1.0, -1.0),
        Vec3::new(-1.0, 3.0, 1.0),
        Vec3::new(1.0, -1.0, 3.0),
    );

    let inv_m = m.invert();
    let ident = m * inv_m;

    assert_relative_eq!(ident.cx.x, 1.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cx.y, 0.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cx.z, 0.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cy.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cy.y, 1.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cy.z, 0.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cz.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cz.y, 0.0, epsilon = 1e-5);
    assert_relative_eq!(ident.cz.z, 1.0, epsilon = 1e-5);

    let v = Vec3::new(1.0, -2.0, 3.0);
    let u = inv_m * (m * v);
    assert_relative_eq!(v.x, u.x, epsilon = 1e-5);
    assert_relative_eq!(v.y, u.y, epsilon = 1e-5);
    assert_relative_eq!(v.z, u.z, epsilon = 1e-5);

    let solved = m.solve(v);
    let direct = inv_m * v;
    assert_relative_eq!(solved.x, direct.x, epsilon = 1e-5);
    assert_relative_eq!(solved.y, direct.y, epsilon = 1e-5);
    assert_relative_eq!(solved.z, direct.z, epsilon = 1e-5);
}

#[test]
fn test_matrix22_ops() {
    let m = Mat22::from_cols(Vec2::new(3.0, 1.0), Vec2::new(-1.0, 3.0));
    let inv_m = m.invert();
    let prod = m * inv_m;

    assert_relative_eq!(prod.cx.x, 1.0, epsilon = 1e-5);
    assert_relative_eq!(prod.cx.y, 0.0, epsilon = 1e-5);
    assert_relative_eq!(prod.cy.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(prod.cy.y, 1.0, epsilon = 1e-5);

    let v2 = Vec2::new(1.0, -2.0);
    let u2 = inv_m * (m * v2);
    assert_relative_eq!(v2.x, u2.x, epsilon = 1e-5);
    assert_relative_eq!(v2.y, u2.y, epsilon = 1e-5);

    let solved = m.solve(v2);
    let direct = inv_m * v2;
    assert_relative_eq!(solved.x, direct.x, epsilon = 1e-5);
    assert_relative_eq!(solved.y, direct.y, epsilon = 1e-5);
}

#[test]
fn test_transform_composition() {
    let axis = Vec3::new(-0.75, 0.5, 1.0).normalize();
    let t1 = Transform {
        p: Vec3::new(-2.0, 3.0, 0.0),
        q: Quat::IDENTITY,
    };
    let t2 = Transform {
        p: Vec3::new(1.0, 0.0, 0.0),
        q: Quat::from_axis_angle(axis, PI),
    };

    let composed = t2 * t1;
    let pt = Vec3::new(2.0, 2.0, 2.0);

    let v1 = t2.transform_point(t1.transform_point(pt));
    let v2 = composed.transform_point(pt);

    assert_relative_eq!(v1.x, v2.x, epsilon = 1e-5);
    assert_relative_eq!(v1.y, v2.y, epsilon = 1e-5);
    assert_relative_eq!(v1.z, v2.z, epsilon = 1e-5);

    let back = t1.inv_transform_point(t1.transform_point(pt));
    assert_relative_eq!(back.x, pt.x, epsilon = 1e-5);
    assert_relative_eq!(back.y, pt.y, epsilon = 1e-5);
    assert_relative_eq!(back.z, pt.z, epsilon = 1e-5);
}

#[test]
fn test_aabb_and_plane() {
    let aabb = AABB::new(
        Vec3::new(-1.0, -2.0, -3.0),
        Vec3::new(1.0, 2.0, 3.0),
    );

    assert!(aabb.contains_point(Vec3::ZERO));
    assert!(aabb.contains_point(Vec3::new(0.5, -1.0, 2.0)));
    assert!(!aabb.contains_point(Vec3::new(2.0, 0.0, 0.0)));

    let aabb2 = AABB::new(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 3.0, 4.0),
    );
    assert!(aabb.overlaps(aabb2));

    let union_box = aabb.union(aabb2);
    assert_eq!(union_box.lower_bound, Vec3::new(-1.0, -2.0, -3.0));
    assert_eq!(union_box.upper_bound, Vec3::new(2.0, 3.0, 4.0));

    // Plane
    let plane = Plane::new(Vec3::Y, 5.0);
    assert_relative_eq!(plane.separation(Vec3::new(0.0, 7.0, 0.0)), 2.0, epsilon = 1e-6);
    assert_relative_eq!(plane.separation(Vec3::new(0.0, 3.0, 0.0)), -2.0, epsilon = 1e-6);
}
