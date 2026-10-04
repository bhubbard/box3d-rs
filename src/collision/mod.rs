// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

pub mod broad_phase;
pub mod dynamic_tree;
pub mod hull;
pub mod manifold;
pub mod sat;
pub mod shape;

pub use broad_phase::{BroadPhase, ShapePair};
pub use dynamic_tree::DynamicTree;
pub use hull::{make_box_hull, make_cube_hull, make_transformed_box_hull, ConvexHull};
pub use manifold::{collide_hulls, collide_shapes, collide_spheres, ContactManifold, ManifoldPoint};
pub use sat::{find_separating_axis, SeparatingAxis};
pub use shape::{Capsule, Shape, ShapeGeometry, Sphere};
