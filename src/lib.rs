// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! # box3d-rs: High-Performance 3D Rigid Body Physics Engine for Rust
//!
//! `box3d-rs` is a pure Rust, zero-native-dependency fork of Erin Catto's authentic [Box3D](https://github.com/erincatto/box3d).
//!
//! ## Core Features
//! - **3D Vector & Transform Math**: Ergonomic `Vec3`, `Quat`, `Mat33`, `Transform`, `AABB`, `Plane`.
//! - **Broadphase Dynamic AABB Tree**: Balanced binary tree with Surface Area Heuristic (SAH) and fattened bounds.
//! - **Narrowphase & SAT Collision**: Exact Separating Axis Theorem (SAT) for convex hulls with Sutherland-Hodgman polygon clipping.
//! - **Rigid Body Dynamics**: Mass and 3x3 inertia tensors, parallel axis theorem, velocity integration.
//! - **Soft Step Constraint Solver**: Sub-stepping solver with compliance, spring-damper formulation, and 2-axis Coulomb friction cones.
//! - **Joints**: Distance, Revolute (hinge), Prismatic (slider), Spherical (ball/socket), and Weld joints.
//! - **Generational Handle IDs**: Memory-safe `WorldId`, `BodyId`, `ShapeId`, `JointId`.

pub mod collision;
pub mod dynamics;
pub mod id;
pub mod math;
pub mod types;
pub mod world;

pub use collision::*;
pub use dynamics::*;
pub use id::*;
pub use math::*;
pub use types::*;
pub use world::World;

/// Helper function to create default world definition
pub fn default_world_def() -> WorldDef {
    WorldDef::default()
}

/// Helper function to create default body definition
pub fn default_body_def() -> BodyDef {
    BodyDef::default()
}

/// Helper function to create default shape definition
pub fn default_shape_def() -> ShapeDef {
    ShapeDef::default()
}

/// Create a physics world with default configuration
pub fn create_world(def: &WorldDef) -> World {
    World::new(*def)
}
