// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Contact manifold generation and narrow-phase collision algorithms.
//!
//! Includes Sutherland-Hodgman polygon clipping and SAT manifold generation.

use crate::collision::hull::ConvexHull;
use crate::collision::sat::{find_separating_axis, SeparatingAxisType};
use crate::collision::shape::{Capsule, ShapeGeometry, Sphere};
use crate::math::{Plane, Transform, Vec2, Vec3};

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ManifoldPoint {
    pub point: Vec3,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    pub separation: f32,
    pub normal_impulse: f32,
    pub tangent_impulse: Vec2,
    pub feature_id: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactManifold {
    pub normal: Vec3,
    pub points: Vec<ManifoldPoint>,
}

impl ContactManifold {
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// Clip a polygon against a clipping plane (Sutherland-Hodgman).
pub fn clip_polygon_against_plane(polygon: &[Vec3], plane: Plane) -> Vec<Vec3> {
    if polygon.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(polygon.len() + 1);
    let mut v1 = polygon[polygon.len() - 1];
    let mut d1 = plane.separation(v1);

    for &v2 in polygon {
        let d2 = plane.separation(v2);

        if d1 <= 0.0 && d2 <= 0.0 {
            out.push(v2);
        } else if d1 <= 0.0 && d2 > 0.0 {
            let frac = d1 / (d1 - d2);
            let intersection = v1 + (v2 - v1) * frac;
            out.push(intersection);
        } else if d2 <= 0.0 && d1 > 0.0 {
            let frac = d1 / (d1 - d2);
            let intersection = v1 + (v2 - v1) * frac;
            out.push(intersection);
            out.push(v2);
        }

        v1 = v2;
        d1 = d2;
    }

    out
}

/// Find incident face on hull most anti-parallel to given reference normal (in hull's local frame)
pub fn find_incident_face(hull: &ConvexHull, ref_normal_in_hull: Vec3) -> usize {
    let mut min_dot = f32::MAX;
    let mut best_face = 0;

    for (face_idx, plane) in hull.planes.iter().enumerate() {
        let dot = plane.normal.dot(ref_normal_in_hull);
        if dot < min_dot {
            min_dot = dot;
            best_face = face_idx;
        }
    }

    best_face
}

/// Get vertices of a face in CCW order
pub fn get_face_vertices(hull: &ConvexHull, face_idx: usize) -> Vec<Vec3> {
    let mut vertices = Vec::new();
    let start_edge = hull.faces[face_idx].edge as usize;
    let mut edge_idx = start_edge;

    loop {
        let edge = &hull.edges[edge_idx];
        vertices.push(hull.points[edge.origin as usize]);
        edge_idx = edge.next as usize;
        if edge_idx == start_edge {
            break;
        }
    }

    vertices
}

/// Sphere vs Sphere collision
pub fn collide_spheres(
    sphere_a: &Sphere,
    xf_a: Transform,
    sphere_b: &Sphere,
    xf_b: Transform,
) -> Option<ContactManifold> {
    let p_a = xf_a.p;
    let p_b = xf_b.p;
    let d = p_b - p_a;
    let dist_sq = d.length_squared();
    let total_radius = sphere_a.radius + sphere_b.radius;

    if dist_sq > total_radius * total_radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-6 {
        d * (1.0 / dist)
    } else {
        Vec3::Y
    };

    let separation = dist - total_radius;
    let contact_point = p_a + normal * (sphere_a.radius + 0.5 * separation);

    Some(ContactManifold {
        normal,
        points: vec![ManifoldPoint {
            point: contact_point,
            anchor_a: contact_point - p_a,
            anchor_b: contact_point - p_b,
            separation,
            normal_impulse: 0.0,
            tangent_impulse: Vec2::ZERO,
            feature_id: 0,
        }],
    })
}

/// Capsule vs Sphere collision
pub fn collide_capsule_and_sphere(
    capsule_a: &Capsule,
    xf_a: Transform,
    sphere_b: &Sphere,
    xf_b: Transform,
) -> Option<ContactManifold> {
    let a1 = xf_a.transform_point(capsule_a.point1);
    let a2 = xf_a.transform_point(capsule_a.point2);
    let p_b = xf_b.p;

    // Closest point on capsule line segment to sphere center
    let ab = a2 - a1;
    let ab_len_sq = ab.length_squared();
    let t = if ab_len_sq > 1e-6 {
        ((p_b - a1).dot(ab) / ab_len_sq).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let closest_a = a1 + ab * t;

    let d = p_b - closest_a;
    let dist_sq = d.length_squared();
    let total_radius = capsule_a.radius + sphere_b.radius;

    if dist_sq > total_radius * total_radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-6 {
        d * (1.0 / dist)
    } else {
        Vec3::Y
    };

    let separation = dist - total_radius;
    let contact_point = closest_a + normal * (capsule_a.radius + 0.5 * separation);

    Some(ContactManifold {
        normal,
        points: vec![ManifoldPoint {
            point: contact_point,
            anchor_a: contact_point - xf_a.p,
            anchor_b: contact_point - p_b,
            separation,
            normal_impulse: 0.0,
            tangent_impulse: Vec2::ZERO,
            feature_id: 0,
        }],
    })
}

/// Convex Hull vs Convex Hull collision using SAT and Sutherland-Hodgman clipping
pub fn collide_hulls(
    hull_a: &ConvexHull,
    xf_a: Transform,
    hull_b: &ConvexHull,
    xf_b: Transform,
) -> Option<ContactManifold> {
    let xf_b_to_a = Transform::inv_mul_transforms(xf_a, xf_b);
    let speculative_dist = crate::math::SPECULATIVE_DISTANCE;

    let axis = find_separating_axis(hull_a, hull_b, xf_b_to_a, speculative_dist)?;

    if axis.separation > speculative_dist {
        return None;
    }

    let mut points = Vec::new();
    let world_normal: Vec3;

    match axis.axis_type {
        SeparatingAxisType::FaceA(face_a) => {
            // Reference face is on Hull A
            let ref_plane = hull_a.planes[face_a];
            let ref_normal_in_b = xf_b_to_a.q.inv_rotate_vec3(ref_plane.normal);
            let inc_face_b = find_incident_face(hull_b, ref_normal_in_b);

            // Get incident polygon in frame of A
            let inc_poly_local = get_face_vertices(hull_b, inc_face_b);
            let mut inc_poly: Vec<Vec3> = inc_poly_local
                .iter()
                .map(|&p| xf_b_to_a.transform_point(p))
                .collect();

            // Clip against side planes of reference face on A
            let start_edge = hull_a.faces[face_a].edge as usize;
            let mut edge_idx = start_edge;
            loop {
                let edge = &hull_a.edges[edge_idx];
                let v1 = hull_a.points[edge.origin as usize];
                let v2 = hull_a.points[hull_a.edges[edge.next as usize].origin as usize];
                let tangent = (v2 - v1).normalize();
                let side_normal = tangent.cross(ref_plane.normal);
                let side_plane = Plane::from_normal_and_point(side_normal, v1);

                inc_poly = clip_polygon_against_plane(&inc_poly, side_plane);
                if inc_poly.is_empty() {
                    break;
                }

                edge_idx = edge.next as usize;
                if edge_idx == start_edge {
                    break;
                }
            }

            // Keep vertices behind reference plane
            world_normal = xf_a.q.rotate_vec3(ref_plane.normal);
            for (idx, p_a) in inc_poly.into_iter().enumerate() {
                let sep = ref_plane.separation(p_a);
                if sep <= speculative_dist {
                    let world_pt = xf_a.transform_point(p_a);
                    points.push(ManifoldPoint {
                        point: world_pt,
                        anchor_a: world_pt - xf_a.p,
                        anchor_b: world_pt - xf_b.p,
                        separation: sep,
                        normal_impulse: 0.0,
                        tangent_impulse: Vec2::ZERO,
                        feature_id: idx as u32,
                    });
                }
            }
        }
        SeparatingAxisType::FaceB(face_b) => {
            // Reference face is on Hull B
            let ref_plane_in_b = hull_b.planes[face_b];
            let ref_normal_in_a = xf_b_to_a.q.rotate_vec3(ref_plane_in_b.normal);
            let inc_face_a = find_incident_face(hull_a, ref_normal_in_a);

            let inc_poly_local = get_face_vertices(hull_a, inc_face_a);
            let mut inc_poly: Vec<Vec3> = inc_poly_local
                .iter()
                .map(|&p| xf_b_to_a.inv_transform_point(p))
                .collect();

            // Clip against side planes of reference face on B
            let start_edge = hull_b.faces[face_b].edge as usize;
            let mut edge_idx = start_edge;
            loop {
                let edge = &hull_b.edges[edge_idx];
                let v1 = hull_b.points[edge.origin as usize];
                let v2 = hull_b.points[hull_b.edges[edge.next as usize].origin as usize];
                let tangent = (v2 - v1).normalize();
                let side_normal = tangent.cross(ref_plane_in_b.normal);
                let side_plane = Plane::from_normal_and_point(side_normal, v1);

                inc_poly = clip_polygon_against_plane(&inc_poly, side_plane);
                if inc_poly.is_empty() {
                    break;
                }

                edge_idx = edge.next as usize;
                if edge_idx == start_edge {
                    break;
                }
            }

            // Normal points from A to B: -world_normal_of_B
            world_normal = -xf_b.q.rotate_vec3(ref_plane_in_b.normal);
            for (idx, p_b) in inc_poly.into_iter().enumerate() {
                let sep = ref_plane_in_b.separation(p_b);
                if sep <= speculative_dist {
                    let world_pt = xf_b.transform_point(p_b);
                    points.push(ManifoldPoint {
                        point: world_pt,
                        anchor_a: world_pt - xf_a.p,
                        anchor_b: world_pt - xf_b.p,
                        separation: sep,
                        normal_impulse: 0.0,
                        tangent_impulse: Vec2::ZERO,
                        feature_id: idx as u32,
                    });
                }
            }
        }
        SeparatingAxisType::Edge(..) => {
            // Edge-edge contact produces 1 contact point at closest segment points
            world_normal = xf_a.q.rotate_vec3(axis.normal);
            let support_a = hull_a.find_support_vertex(axis.normal);
            let pt_a = xf_a.transform_point(hull_a.points[support_a]);
            points.push(ManifoldPoint {
                point: pt_a,
                anchor_a: pt_a - xf_a.p,
                anchor_b: pt_a - xf_b.p,
                separation: axis.separation,
                normal_impulse: 0.0,
                tangent_impulse: Vec2::ZERO,
                feature_id: 0,
            });
        }
    }

    if points.is_empty() {
        None
    } else {
        Some(ContactManifold {
            normal: world_normal,
            points,
        })
    }
}

/// Convex Hull vs Sphere collision
pub fn collide_hull_and_sphere(
    hull_a: &ConvexHull,
    xf_a: Transform,
    sphere_b: &Sphere,
    xf_b: Transform,
) -> Option<ContactManifold> {
    let center_in_a = xf_a.inv_transform_point(xf_b.p);
    let speculative_dist = crate::math::SPECULATIVE_DISTANCE;

    let mut best_face = 0;
    let mut max_sep = f32::NEG_INFINITY;

    for (i, plane) in hull_a.planes.iter().enumerate() {
        let sep = plane.separation(center_in_a);
        if sep > max_sep {
            max_sep = sep;
            best_face = i;
        }
    }

    if max_sep > sphere_b.radius + speculative_dist {
        return None;
    }

    let plane = hull_a.planes[best_face];

    let (normal_in_a, separation, contact_point_in_a) = if max_sep <= 0.0 {
        // Sphere center is inside the hull
        (
            plane.normal,
            max_sep - sphere_b.radius,
            center_in_a - plane.normal * max_sep,
        )
    } else {
        // Sphere center is outside the hull
        let proj = center_in_a - plane.normal * max_sep;
        let face_verts = get_face_vertices(hull_a, best_face);
        let mut closest_point = proj;

        let n_verts = face_verts.len();
        for i in 0..n_verts {
            let v1 = face_verts[i];
            let v2 = face_verts[(i + 1) % n_verts];
            let edge_dir = v2 - v1;
            let edge_len_sq = edge_dir.length_squared();
            let side_normal = edge_dir.cross(plane.normal).normalize();

            let side_sep = (proj - v1).dot(side_normal);
            if side_sep > 0.0 {
                let t = if edge_len_sq > 1e-6 {
                    ((center_in_a - v1).dot(edge_dir) / edge_len_sq).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                closest_point = v1 + edge_dir * t;
                break;
            }
        }

        let delta = center_in_a - closest_point;
        let dist = delta.length();
        if dist > sphere_b.radius + speculative_dist {
            return None;
        }

        let normal = if dist > 1e-6 {
            delta * (1.0 / dist)
        } else {
            plane.normal
        };

        (normal, dist - sphere_b.radius, closest_point)
    };

    let world_normal = xf_a.q.rotate_vec3(normal_in_a);
    let world_point = xf_a.transform_point(contact_point_in_a);

    Some(ContactManifold {
        normal: world_normal,
        points: vec![ManifoldPoint {
            point: world_point,
            anchor_a: world_point - xf_a.p,
            anchor_b: world_point - xf_b.p,
            separation,
            normal_impulse: 0.0,
            tangent_impulse: Vec2::ZERO,
            feature_id: 0,
        }],
    })
}

/// Generic narrow-phase dispatch between any two shapes
pub fn collide_shapes(
    geom_a: &ShapeGeometry,
    xf_a: Transform,
    geom_b: &ShapeGeometry,
    xf_b: Transform,
) -> Option<ContactManifold> {
    match (geom_a, geom_b) {
        (ShapeGeometry::Sphere(s_a), ShapeGeometry::Sphere(s_b)) => {
            collide_spheres(s_a, xf_a, s_b, xf_b)
        }
        (ShapeGeometry::Capsule(c_a), ShapeGeometry::Sphere(s_b)) => {
            collide_capsule_and_sphere(c_a, xf_a, s_b, xf_b)
        }
        (ShapeGeometry::Sphere(s_a), ShapeGeometry::Capsule(c_b)) => {
            let mut manifold = collide_capsule_and_sphere(c_b, xf_b, s_a, xf_a)?;
            manifold.normal = -manifold.normal;
            for p in &mut manifold.points {
                std::mem::swap(&mut p.anchor_a, &mut p.anchor_b);
            }
            Some(manifold)
        }
        (ShapeGeometry::Hull(h_a), ShapeGeometry::Sphere(s_b)) => {
            collide_hull_and_sphere(h_a, xf_a, s_b, xf_b)
        }
        (ShapeGeometry::Sphere(s_a), ShapeGeometry::Hull(h_b)) => {
            let mut manifold = collide_hull_and_sphere(h_b, xf_b, s_a, xf_a)?;
            manifold.normal = -manifold.normal;
            for p in &mut manifold.points {
                std::mem::swap(&mut p.anchor_a, &mut p.anchor_b);
            }
            Some(manifold)
        }
        (ShapeGeometry::Hull(h_a), ShapeGeometry::Hull(h_b)) => {
            collide_hulls(h_a, xf_a, h_b, xf_b)
        }
        _ => None,
    }
}
