// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Polyhedral Convex Hull representation and generators (Box, Cube, General Polyhedra).
//!
//! Stores faces, half-edges, planes, vertices, and pre-computed mass properties.

use crate::math::{AABB, Mat33, Plane, Transform, Vec3};
use crate::types::MassData;

/// Vertex referencing its primary outgoing half-edge.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HullVertex {
    pub edge: u8,
}

/// Half-edge in doubly-connected edge list (DCEL) format.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HullHalfEdge {
    /// Next CCW edge on this face
    pub next: u8,
    /// Twin edge belonging to the adjacent face
    pub twin: u8,
    /// Origin vertex index
    pub origin: u8,
    /// Face index to the left of this half-edge
    pub face: u8,
}

/// Polygon face on a convex hull.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HullFace {
    /// An arbitrary half-edge index on this face
    pub edge: u8,
}

/// A Polyhedral Convex Hull geometry with topological connectivity.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ConvexHull {
    pub vertices: Vec<HullVertex>,
    pub points: Vec<Vec3>,
    pub edges: Vec<HullHalfEdge>,
    pub planes: Vec<Plane>,
    pub faces: Vec<HullFace>,
    pub aabb: AABB,
    pub volume: f32,
    pub surface_area: f32,
    pub center_of_mass: Vec3,
    pub local_inertia: Mat33,
}

impl ConvexHull {
    /// Find support vertex index in given direction
    #[inline]
    pub fn find_support_vertex(&self, direction: Vec3) -> usize {
        let mut max_dot = f32::NEG_INFINITY;
        let mut best_index = 0;
        for (i, p) in self.points.iter().enumerate() {
            let dot = p.dot(direction);
            if dot > max_dot {
                max_dot = dot;
                best_index = i;
            }
        }
        best_index
    }

    /// Compute mass data with given density
    pub fn compute_mass_data(&self, density: f32) -> MassData {
        let mass = density * self.volume;
        let inertia = self.local_inertia * density;
        MassData::new(mass, self.center_of_mass, inertia)
    }
}

/// Constant topology for box hull (8 vertices, 24 edges, 6 faces)
const BOX_VERTICES: [HullVertex; 8] = [
    HullVertex { edge: 8 },
    HullVertex { edge: 1 },
    HullVertex { edge: 0 },
    HullVertex { edge: 9 },
    HullVertex { edge: 13 },
    HullVertex { edge: 3 },
    HullVertex { edge: 5 },
    HullVertex { edge: 11 },
];

const BOX_EDGES: [HullHalfEdge; 24] = [
    HullHalfEdge { next: 2, twin: 1, origin: 2, face: 0 },
    HullHalfEdge { next: 17, twin: 0, origin: 1, face: 5 },
    HullHalfEdge { next: 4, twin: 3, origin: 1, face: 0 },
    HullHalfEdge { next: 20, twin: 2, origin: 5, face: 3 },
    HullHalfEdge { next: 6, twin: 5, origin: 5, face: 0 },
    HullHalfEdge { next: 23, twin: 4, origin: 6, face: 4 },
    HullHalfEdge { next: 0, twin: 7, origin: 6, face: 0 },
    HullHalfEdge { next: 18, twin: 6, origin: 2, face: 2 },
    HullHalfEdge { next: 10, twin: 9, origin: 0, face: 1 },
    HullHalfEdge { next: 21, twin: 8, origin: 3, face: 5 },
    HullHalfEdge { next: 12, twin: 11, origin: 3, face: 1 },
    HullHalfEdge { next: 16, twin: 10, origin: 7, face: 2 },
    HullHalfEdge { next: 14, twin: 13, origin: 7, face: 1 },
    HullHalfEdge { next: 19, twin: 12, origin: 4, face: 4 },
    HullHalfEdge { next: 8, twin: 15, origin: 4, face: 1 },
    HullHalfEdge { next: 22, twin: 14, origin: 0, face: 3 },
    HullHalfEdge { next: 7, twin: 17, origin: 3, face: 2 },
    HullHalfEdge { next: 9, twin: 16, origin: 2, face: 5 },
    HullHalfEdge { next: 11, twin: 19, origin: 6, face: 2 },
    HullHalfEdge { next: 5, twin: 18, origin: 7, face: 4 },
    HullHalfEdge { next: 15, twin: 21, origin: 1, face: 3 },
    HullHalfEdge { next: 1, twin: 20, origin: 0, face: 5 },
    HullHalfEdge { next: 3, twin: 23, origin: 4, face: 3 },
    HullHalfEdge { next: 13, twin: 22, origin: 5, face: 4 },
];

const BOX_FACES: [HullFace; 6] = [
    HullFace { edge: 0 },
    HullFace { edge: 8 },
    HullFace { edge: 16 },
    HullFace { edge: 20 },
    HullFace { edge: 19 },
    HullFace { edge: 21 },
];

/// Create a transformed box convex hull.
pub fn make_transformed_box_hull(hx: f32, hy: f32, hz: f32, transform: Transform) -> ConvexHull {
    let min_h = 0.2 * crate::math::LINEAR_SLOP;
    let h = Vec3::new(hx.max(min_h), hy.max(min_h), hz.max(min_h));

    let local_aabb = AABB::new(-h, h);
    let aabb = local_aabb.transform(transform);
    let volume = 8.0 * h.x * h.y * h.z;
    let surface_area = 8.0 * (h.x * h.y + h.x * h.z + h.y * h.z);

    // Box unit inertia (density = 1.0)
    let box_inertia = Mat33::box_inertia(volume, h);
    let central_inertia = Mat33::rotate_inertia(transform.q, box_inertia);

    let lower = -h;
    let upper = h;

    // 6 Planes transformed into world coordinates: -X, +X, -Y, +Y, -Z, +Z
    let planes = vec![
        Plane::from_normal_and_point(-Vec3::X, lower).transform(transform),
        Plane::from_normal_and_point(Vec3::X, upper).transform(transform),
        Plane::from_normal_and_point(-Vec3::Y, lower).transform(transform),
        Plane::from_normal_and_point(Vec3::Y, upper).transform(transform),
        Plane::from_normal_and_point(-Vec3::Z, lower).transform(transform),
        Plane::from_normal_and_point(Vec3::Z, upper).transform(transform),
    ];

    // 8 Box points in local space transformed
    let local_points = [
        Vec3::new(h.x, h.y, h.z),
        Vec3::new(-h.x, h.y, h.z),
        Vec3::new(-h.x, -h.y, h.z),
        Vec3::new(h.x, -h.y, h.z),
        Vec3::new(h.x, h.y, -h.z),
        Vec3::new(-h.x, h.y, -h.z),
        Vec3::new(-h.x, -h.y, -h.z),
        Vec3::new(h.x, -h.y, -h.z),
    ];

    let points: Vec<Vec3> = local_points.iter().map(|&p| transform.transform_point(p)).collect();

    ConvexHull {
        vertices: BOX_VERTICES.to_vec(),
        points,
        edges: BOX_EDGES.to_vec(),
        planes,
        faces: BOX_FACES.to_vec(),
        aabb,
        volume,
        surface_area,
        center_of_mass: transform.p,
        local_inertia: central_inertia,
    }
}

/// Create a box convex hull centered at origin with half-extents (hx, hy, hz).
pub fn make_box_hull(hx: f32, hy: f32, hz: f32) -> ConvexHull {
    make_transformed_box_hull(hx, hy, hz, Transform::IDENTITY)
}

/// Create a cube convex hull centered at origin with equal half-extent `half_width`.
pub fn make_cube_hull(half_width: f32) -> ConvexHull {
    make_box_hull(half_width, half_width, half_width)
}
