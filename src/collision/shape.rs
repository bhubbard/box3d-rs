// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Collision shapes: Sphere, Capsule, and Convex Hull.

use std::f32::consts::PI;

use crate::collision::hull::ConvexHull;
use crate::id::{BodyId, ShapeId};
use crate::math::{AABB, Mat33, Transform, Vec3};
use crate::types::{MassData, ShapeDef, SurfaceMaterial};

/// Sphere geometry centered at shape local origin.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sphere {
    pub radius: f32,
}

impl Sphere {
    pub const fn new(radius: f32) -> Self {
        Self { radius }
    }

    pub fn compute_aabb(&self, xf: Transform) -> AABB {
        let r = Vec3::new(self.radius, self.radius, self.radius);
        AABB::new(xf.p - r, xf.p + r)
    }

    pub fn compute_mass_data(&self, density: f32) -> MassData {
        let r = self.radius;
        let volume = (4.0 / 3.0) * PI * r * r * r;
        let mass = density * volume;
        let inertia_val = 0.4 * mass * r * r;
        let inertia = Mat33::from_diagonal(inertia_val, inertia_val, inertia_val);
        MassData::new(mass, Vec3::ZERO, inertia)
    }
}

/// Capsule geometry extending along a segment from point1 to point2 with radius.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Capsule {
    pub point1: Vec3,
    pub point2: Vec3,
    pub radius: f32,
}

impl Capsule {
    pub const fn new(point1: Vec3, point2: Vec3, radius: f32) -> Self {
        Self {
            point1,
            point2,
            radius,
        }
    }

    pub fn compute_aabb(&self, xf: Transform) -> AABB {
        let p1 = xf.transform_point(self.point1);
        let p2 = xf.transform_point(self.point2);
        let r = Vec3::new(self.radius, self.radius, self.radius);
        let lower = p1.min(p2) - r;
        let upper = p1.max(p2) + r;
        AABB::new(lower, upper)
    }

    pub fn compute_mass_data(&self, density: f32) -> MassData {
        let length = (self.point2 - self.point1).length();
        let r = self.radius;
        let cyl_vol = PI * r * r * length;
        let sph_vol = (4.0 / 3.0) * PI * r * r * r;
        let mass = density * (cyl_vol + sph_vol);

        let center = (self.point1 + self.point2) * 0.5;
        // Approximate inertia as bounding cylinder
        let h = length + 2.0 * r;
        let izz = 0.5 * mass * r * r;
        let ixx = mass * (3.0 * r * r + h * h) / 12.0;
        let inertia = Mat33::from_diagonal(ixx, ixx, izz);
        MassData::new(mass, center, inertia)
    }
}

/// Enum containing any supported collision shape geometry.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ShapeGeometry {
    Sphere(Sphere),
    Capsule(Capsule),
    Hull(ConvexHull),
}

impl ShapeGeometry {
    pub fn compute_aabb(&self, xf: Transform) -> AABB {
        match self {
            Self::Sphere(s) => s.compute_aabb(xf),
            Self::Capsule(c) => c.compute_aabb(xf),
            Self::Hull(h) => h.aabb.transform(xf),
        }
    }

    pub fn compute_mass_data(&self, density: f32) -> MassData {
        match self {
            Self::Sphere(s) => s.compute_mass_data(density),
            Self::Capsule(c) => c.compute_mass_data(density),
            Self::Hull(h) => h.compute_mass_data(density),
        }
    }
}

/// An instantiated shape attached to a rigid body in the physics world.
#[derive(Clone, Debug)]
pub struct Shape {
    pub id: ShapeId,
    pub body_id: BodyId,
    pub geometry: ShapeGeometry,
    pub density: f32,
    pub material: SurfaceMaterial,
    pub is_sensor: bool,
    pub fat_aabb: AABB,
    pub proxy_id: i32,
}

impl Shape {
    pub fn new(id: ShapeId, body_id: BodyId, geometry: ShapeGeometry, def: &ShapeDef) -> Self {
        Self {
            id,
            body_id,
            geometry,
            density: def.density,
            material: def.material,
            is_sensor: def.is_sensor,
            fat_aabb: AABB::default(),
            proxy_id: -1,
        }
    }

    pub fn compute_mass_data(&self) -> MassData {
        if self.density <= 0.0 || self.is_sensor {
            MassData::default()
        } else {
            self.geometry.compute_mass_data(self.density)
        }
    }
}
