// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Separating Axis Theorem (SAT) for convex polyhedra.
//!
//! Evaluates face axes and edge-edge cross product axes to find the minimum penetration
//! or detect separation.

use crate::collision::hull::ConvexHull;
use crate::math::{Transform, Vec3};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SeparatingAxisType {
    FaceA(usize),
    FaceB(usize),
    Edge(usize, usize),
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SeparatingAxis {
    pub axis_type: SeparatingAxisType,
    pub normal: Vec3,
    pub separation: f32,
}

/// Query face planes of Hull A against support points of Hull B (all in frame of A)
pub fn query_face_directions_a(
    hull_a: &ConvexHull,
    hull_b: &ConvexHull,
    xf_b_to_a: Transform,
    speculative_distance: f32,
) -> Option<SeparatingAxis> {
    let mut max_sep = f32::NEG_INFINITY;
    let mut best_axis = None;

    for (face_idx, plane) in hull_a.planes.iter().enumerate() {
        // Plane normal in frame B
        let dir_in_b = -xf_b_to_a.q.inv_rotate_vec3(plane.normal);
        let support_idx = hull_b.find_support_vertex(dir_in_b);
        let support_point_in_a = xf_b_to_a.transform_point(hull_b.points[support_idx]);
        let sep = plane.separation(support_point_in_a);

        if sep > speculative_distance {
            // Shapes are definitely separated
            return Some(SeparatingAxis {
                axis_type: SeparatingAxisType::FaceA(face_idx),
                normal: plane.normal,
                separation: sep,
            });
        }

        if sep > max_sep {
            max_sep = sep;
            best_axis = Some(SeparatingAxis {
                axis_type: SeparatingAxisType::FaceA(face_idx),
                normal: plane.normal,
                separation: sep,
            });
        }
    }

    best_axis
}

/// Query face planes of Hull B against support points of Hull A (all in frame of A)
pub fn query_face_directions_b(
    hull_a: &ConvexHull,
    hull_b: &ConvexHull,
    xf_b_to_a: Transform,
    speculative_distance: f32,
) -> Option<SeparatingAxis> {
    let mut max_sep = f32::NEG_INFINITY;
    let mut best_axis = None;

    for (face_idx, plane_b) in hull_b.planes.iter().enumerate() {
        // Transform Hull B's plane into frame of A
        let plane_in_a = plane_b.transform(xf_b_to_a);
        let dir_in_a = -plane_in_a.normal;
        let support_idx = hull_a.find_support_vertex(dir_in_a);
        let support_point = hull_a.points[support_idx];
        let sep = plane_in_a.separation(support_point);

        if sep > speculative_distance {
            return Some(SeparatingAxis {
                axis_type: SeparatingAxisType::FaceB(face_idx),
                normal: plane_in_a.normal,
                separation: sep,
            });
        }

        if sep > max_sep {
            max_sep = sep;
            best_axis = Some(SeparatingAxis {
                axis_type: SeparatingAxisType::FaceB(face_idx),
                normal: plane_in_a.normal,
                separation: sep,
            });
        }
    }

    best_axis
}

/// Query edge-edge cross products between Hull A and Hull B
pub fn query_edge_directions(
    hull_a: &ConvexHull,
    hull_b: &ConvexHull,
    xf_b_to_a: Transform,
    speculative_distance: f32,
) -> Option<SeparatingAxis> {
    let mut max_sep = f32::NEG_INFINITY;
    let mut best_axis = None;

    // Transform B points into frame A
    let points_b_in_a: Vec<Vec3> = hull_b.points.iter().map(|&p| xf_b_to_a.transform_point(p)).collect();

    for (edge_idx_a, edge_a) in hull_a.edges.iter().enumerate() {
        let p_a1 = hull_a.points[edge_a.origin as usize];
        let p_a2 = hull_a.points[hull_a.edges[edge_a.next as usize].origin as usize];
        let d_a = p_a2 - p_a1;

        for (edge_idx_b, edge_b) in hull_b.edges.iter().enumerate() {
            let p_b1 = points_b_in_a[edge_b.origin as usize];
            let p_b2 = points_b_in_a[hull_b.edges[edge_b.next as usize].origin as usize];
            let d_b = p_b2 - p_b1;

            let axis = d_a.cross(d_b);
            let len_sq = axis.length_squared();
            if len_sq < 1e-6 {
                continue; // Parallel edges
            }

            let mut normal = axis.normalize();
            // Ensure normal points from A to B
            if normal.dot(p_b1 - p_a1) < 0.0 {
                normal = -normal;
            }

            // Project A and B onto normal
            let support_a = hull_a.find_support_vertex(normal);
            let support_b = hull_b.find_support_vertex(-xf_b_to_a.q.inv_rotate_vec3(normal));
            let pt_b = xf_b_to_a.transform_point(hull_b.points[support_b]);
            let sep = normal.dot(pt_b - hull_a.points[support_a]);

            if sep > speculative_distance {
                return Some(SeparatingAxis {
                    axis_type: SeparatingAxisType::Edge(edge_idx_a, edge_idx_b),
                    normal,
                    separation: sep,
                });
            }

            if sep > max_sep {
                max_sep = sep;
                best_axis = Some(SeparatingAxis {
                    axis_type: SeparatingAxisType::Edge(edge_idx_a, edge_idx_b),
                    normal,
                    separation: sep,
                });
            }
        }
    }

    best_axis
}

/// Find optimal separating axis between two convex hulls in frame of Hull A
pub fn find_separating_axis(
    hull_a: &ConvexHull,
    hull_b: &ConvexHull,
    xf_b_to_a: Transform,
    speculative_distance: f32,
) -> Option<SeparatingAxis> {
    let axis_a = query_face_directions_a(hull_a, hull_b, xf_b_to_a, speculative_distance)?;
    if axis_a.separation > speculative_distance {
        return Some(axis_a);
    }

    let axis_b = query_face_directions_b(hull_a, hull_b, xf_b_to_a, speculative_distance)?;
    if axis_b.separation > speculative_distance {
        return Some(axis_b);
    }

    // Return the axis with maximum separation (least penetration)
    if axis_a.separation >= axis_b.separation {
        Some(axis_a)
    } else {
        Some(axis_b)
    }
}
