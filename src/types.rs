// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Fundamental definitions, configurations, and surface materials.

use crate::math::{Mat33, Quat, Vec3};

/// The body simulation type.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BodyType {
    /// Zero mass, zero velocity, moved only programmatically. Never collides with static.
    #[default]
    Static = 0,
    /// Zero mass, non-zero velocity set by user, moved by solver.
    Kinematic = 1,
    /// Positive mass, moved by forces and collision impulses.
    Dynamic = 2,
}

/// World definition used to initialize a physics world.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WorldDef {
    /// Gravity vector. Convention uses +Y as up, so default is (0, -10, 0).
    pub gravity: Vec3,
    /// Contact constraint spring frequency in Hertz.
    pub contact_hertz: f32,
    /// Contact constraint damping ratio (0 to 1).
    pub contact_damping_ratio: f32,
    /// Maximum contact constraint push out speed (m/s).
    pub contact_speed: f32,
    /// Contact recycling distance (meters).
    pub contact_recycle_distance: f32,
    /// Maximum linear velocity clamp (m/s).
    pub max_linear_speed: f32,
    /// Enable sleeping for inactive bodies to save CPU.
    pub enable_sleep: bool,
    /// Enable warm starting for constraint impulses.
    pub enable_warm_starting: bool,
}

impl Default for WorldDef {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -10.0, 0.0),
            contact_hertz: 30.0,
            contact_damping_ratio: 10.0,
            contact_speed: 3.0,
            contact_recycle_distance: 0.02,
            max_linear_speed: 400.0,
            enable_sleep: true,
            enable_warm_starting: true,
        }
    }
}

/// Rigid body definition.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BodyDef {
    pub body_type: BodyType,
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub enable_sleep: bool,
    pub is_awake: bool,
}

impl Default for BodyDef {
    fn default() -> Self {
        Self {
            body_type: BodyType::Static,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.05,
            gravity_scale: 1.0,
            enable_sleep: true,
            is_awake: true,
        }
    }
}

/// Collision filtering rules.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Filter {
    pub category_bits: u32,
    pub mask_bits: u32,
    pub group_index: i32,
}

impl Default for Filter {
    fn default() -> Self {
        Self {
            category_bits: 0x00000001,
            mask_bits: 0xFFFFFFFF,
            group_index: 0,
        }
    }
}

impl Filter {
    #[inline]
    pub fn should_collide(self, other: Self) -> bool {
        if self.group_index != 0 && self.group_index == other.group_index {
            return self.group_index > 0;
        }
        (self.mask_bits & other.category_bits) != 0 && (self.category_bits & other.mask_bits) != 0
    }
}

/// Surface physical material.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SurfaceMaterial {
    /// Coulomb friction coefficient (0 = frictionless, 1 = high friction)
    pub friction: f32,
    /// Restitution (bounciness, 0 = inelastic, 1 = perfectly elastic)
    pub restitution: f32,
    /// Rolling resistance coefficient
    pub rolling_resistance: f32,
}

impl Default for SurfaceMaterial {
    fn default() -> Self {
        Self {
            friction: 0.6,
            restitution: 0.0,
            rolling_resistance: 0.0,
        }
    }
}

/// Shape definition attached to a rigid body.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ShapeDef {
    /// Density in kg/m^3. Non-zero density produces positive mass for dynamic bodies.
    pub density: f32,
    pub material: SurfaceMaterial,
    pub is_sensor: bool,
    pub filter: Filter,
}

impl Default for ShapeDef {
    fn default() -> Self {
        Self {
            density: 1.0,
            material: SurfaceMaterial::default(),
            is_sensor: false,
            filter: Filter::default(),
        }
    }
}

/// Mass and rotational inertia properties.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MassData {
    pub mass: f32,
    pub center: Vec3,
    pub inertia: Mat33,
    pub inv_mass: f32,
    pub inv_inertia: Mat33,
}

impl Default for MassData {
    fn default() -> Self {
        Self {
            mass: 0.0,
            center: Vec3::ZERO,
            inertia: Mat33::ZERO,
            inv_mass: 0.0,
            inv_inertia: Mat33::ZERO,
        }
    }
}

impl MassData {
    pub fn new(mass: f32, center: Vec3, inertia: Mat33) -> Self {
        let (inv_mass, inv_inertia) = if mass > 0.0 {
            (1.0 / mass, inertia.invert())
        } else {
            (0.0, Mat33::ZERO)
        };
        Self {
            mass,
            center,
            inertia,
            inv_mass,
            inv_inertia,
        }
    }
}
