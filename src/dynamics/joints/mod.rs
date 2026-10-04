// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Joint constraints: Distance, Revolute (hinge), Prismatic (slider), Spherical (ball/socket), and Weld.

pub use crate::id::{BodyId, JointId};
use crate::math::{Quat, Vec3};

#[derive(Clone, Debug)]
pub struct DistanceJointDef {
    pub body_id_a: BodyId,
    pub body_id_b: BodyId,
    pub local_anchor_a: Vec3,
    pub local_anchor_b: Vec3,
    pub length: f32,
    pub min_length: f32,
    pub max_length: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
}

impl Default for DistanceJointDef {
    fn default() -> Self {
        Self {
            body_id_a: BodyId::NULL,
            body_id_b: BodyId::NULL,
            local_anchor_a: Vec3::ZERO,
            local_anchor_b: Vec3::ZERO,
            length: 1.0,
            min_length: 0.0,
            max_length: f32::MAX,
            hertz: 0.0,
            damping_ratio: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct RevoluteJointDef {
    pub body_id_a: BodyId,
    pub body_id_b: BodyId,
    pub local_anchor_a: Vec3,
    pub local_anchor_b: Vec3,
    pub local_axis_a: Vec3,
    pub enable_limit: bool,
    pub lower_angle: f32,
    pub upper_angle: f32,
    pub enable_motor: bool,
    pub motor_speed: f32,
    pub max_motor_torque: f32,
}

impl Default for RevoluteJointDef {
    fn default() -> Self {
        Self {
            body_id_a: BodyId::NULL,
            body_id_b: BodyId::NULL,
            local_anchor_a: Vec3::ZERO,
            local_anchor_b: Vec3::ZERO,
            local_axis_a: Vec3::Z,
            enable_limit: false,
            lower_angle: 0.0,
            upper_angle: 0.0,
            enable_motor: false,
            motor_speed: 0.0,
            max_motor_torque: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PrismaticJointDef {
    pub body_id_a: BodyId,
    pub body_id_b: BodyId,
    pub local_anchor_a: Vec3,
    pub local_anchor_b: Vec3,
    pub local_axis_a: Vec3,
    pub enable_limit: bool,
    pub lower_translation: f32,
    pub upper_translation: f32,
    pub enable_motor: bool,
    pub motor_speed: f32,
    pub max_motor_force: f32,
}

impl Default for PrismaticJointDef {
    fn default() -> Self {
        Self {
            body_id_a: BodyId::NULL,
            body_id_b: BodyId::NULL,
            local_anchor_a: Vec3::ZERO,
            local_anchor_b: Vec3::ZERO,
            local_axis_a: Vec3::X,
            enable_limit: false,
            lower_translation: 0.0,
            upper_translation: 0.0,
            enable_motor: false,
            motor_speed: 0.0,
            max_motor_force: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SphericalJointDef {
    pub body_id_a: BodyId,
    pub body_id_b: BodyId,
    pub local_anchor_a: Vec3,
    pub local_anchor_b: Vec3,
}

impl Default for SphericalJointDef {
    fn default() -> Self {
        Self {
            body_id_a: BodyId::NULL,
            body_id_b: BodyId::NULL,
            local_anchor_a: Vec3::ZERO,
            local_anchor_b: Vec3::ZERO,
        }
    }
}

#[derive(Clone, Debug)]
pub struct WeldJointDef {
    pub body_id_a: BodyId,
    pub body_id_b: BodyId,
    pub local_anchor_a: Vec3,
    pub local_anchor_b: Vec3,
    pub reference_rotation: Quat,
}

impl Default for WeldJointDef {
    fn default() -> Self {
        Self {
            body_id_a: BodyId::NULL,
            body_id_b: BodyId::NULL,
            local_anchor_a: Vec3::ZERO,
            local_anchor_b: Vec3::ZERO,
            reference_rotation: Quat::IDENTITY,
        }
    }
}

#[derive(Clone, Debug)]
pub enum JointKind {
    Distance(DistanceJointDef),
    Revolute(RevoluteJointDef),
    Prismatic(PrismaticJointDef),
    Spherical(SphericalJointDef),
    Weld(WeldJointDef),
}

#[derive(Clone, Debug)]
pub struct Joint {
    pub id: JointId,
    pub body_id_a: BodyId,
    pub body_id_b: BodyId,
    pub kind: JointKind,
    pub impulse: Vec3,
}

impl Joint {
    pub fn new(id: JointId, kind: JointKind) -> Self {
        let (body_id_a, body_id_b) = match &kind {
            JointKind::Distance(d) => (d.body_id_a, d.body_id_b),
            JointKind::Revolute(r) => (r.body_id_a, r.body_id_b),
            JointKind::Prismatic(p) => (p.body_id_a, p.body_id_b),
            JointKind::Spherical(s) => (s.body_id_a, s.body_id_b),
            JointKind::Weld(w) => (w.body_id_a, w.body_id_b),
        };
        Self {
            id,
            body_id_a,
            body_id_b,
            kind,
            impulse: Vec3::ZERO,
        }
    }
}
