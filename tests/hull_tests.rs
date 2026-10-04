// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

use approx::assert_relative_eq;
use box3d::collision::hull::*;
use box3d::math::*;

#[test]
fn test_cube_hull_topology_and_euler() {
    let hull = make_cube_hull(1.0);

    assert_eq!(hull.vertices.len(), 8);
    assert_eq!(hull.edges.len(), 24);
    assert_eq!(hull.faces.len(), 6);

    // Euler's characteristic for convex polyhedron: V - E/2 + F = 2
    let v = hull.vertices.len() as i32;
    let e = (hull.edges.len() / 2) as i32;
    let f = hull.faces.len() as i32;
    assert_eq!(v - e + f, 2);

    // Volume of [-1, 1]^3 = 2^3 = 8
    assert_relative_eq!(hull.volume, 8.0, epsilon = 1e-5);

    // Surface area of [-1, 1]^3 = 6 * (2 * 2) = 24
    assert_relative_eq!(hull.surface_area, 24.0, epsilon = 1e-5);

    // Center of mass at origin
    assert_relative_eq!(hull.center_of_mass.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(hull.center_of_mass.y, 0.0, epsilon = 1e-5);
    assert_relative_eq!(hull.center_of_mass.z, 0.0, epsilon = 1e-5);

    // Diagonal elements of inertia tensor for solid box: (1/12)*M*(h_y^2 + h_z^2)*4 = (1/3)*M*(hy^2+hz^2)
    // For unit density: M = 8.0, hy = 1.0, hz = 1.0 -> Ixx = (1/3)*8*(1+1) = 16/3 = 5.333333
    assert_relative_eq!(hull.local_inertia.cx.x, 16.0 / 3.0, epsilon = 1e-4);
    assert_relative_eq!(hull.local_inertia.cy.y, 16.0 / 3.0, epsilon = 1e-4);
    assert_relative_eq!(hull.local_inertia.cz.z, 16.0 / 3.0, epsilon = 1e-4);

    // Off-diagonal elements should be 0 for symmetric box
    assert_relative_eq!(hull.local_inertia.cx.y, 0.0, epsilon = 1e-5);
    assert_relative_eq!(hull.local_inertia.cx.z, 0.0, epsilon = 1e-5);
    assert_relative_eq!(hull.local_inertia.cy.z, 0.0, epsilon = 1e-5);
}

#[test]
fn test_hull_support_vertex() {
    let hull = make_box_hull(2.0, 3.0, 4.0);

    // Cardinal directions
    let sup_px = hull.find_support_vertex(Vec3::X);
    assert_relative_eq!(hull.points[sup_px].x, 2.0, epsilon = 1e-5);

    let sup_nx = hull.find_support_vertex(-Vec3::X);
    assert_relative_eq!(hull.points[sup_nx].x, -2.0, epsilon = 1e-5);

    let sup_py = hull.find_support_vertex(Vec3::Y);
    assert_relative_eq!(hull.points[sup_py].y, 3.0, epsilon = 1e-5);

    let sup_pz = hull.find_support_vertex(Vec3::Z);
    assert_relative_eq!(hull.points[sup_pz].z, 4.0, epsilon = 1e-5);

    // Diagonal direction (+X, +Y, +Z)
    let diag = Vec3::new(1.0, 1.0, 1.0).normalize();
    let sup_diag = hull.find_support_vertex(diag);
    assert_relative_eq!(hull.points[sup_diag].x, 2.0, epsilon = 1e-5);
    assert_relative_eq!(hull.points[sup_diag].y, 3.0, epsilon = 1e-5);
    assert_relative_eq!(hull.points[sup_diag].z, 4.0, epsilon = 1e-5);
}
