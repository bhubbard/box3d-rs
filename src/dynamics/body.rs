// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Rigid body dynamics, forces, torques, velocities, and sleeping.

use crate::id::{BodyId, ShapeId};
use crate::math::{Mat33, Quat, Transform, Vec3};
use crate::types::{BodyDef, BodyType, MassData};

#[derive(Clone, Debug)]
pub struct RigidBody {
    pub id: BodyId,
    pub body_type: BodyType,
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub delta_position: Vec3,
    pub delta_rotation: Quat,
    pub force: Vec3,
    pub torque: Vec3,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub mass: f32,
    pub inv_mass: f32,
    pub local_center: Vec3,
    pub local_inertia: Mat33,
    pub inv_local_inertia: Mat33,
    pub inv_world_inertia: Mat33,
    pub shapes: Vec<ShapeId>,
    pub is_awake: bool,
    pub enable_sleep: bool,
    pub sleep_time: f32,
}

impl RigidBody {
    pub fn new(id: BodyId, def: &BodyDef) -> Self {
        Self {
            id,
            body_type: def.body_type,
            position: def.position,
            rotation: def.rotation.normalize(),
            linear_velocity: def.linear_velocity,
            angular_velocity: def.angular_velocity,
            delta_position: Vec3::ZERO,
            delta_rotation: Quat::IDENTITY,
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            linear_damping: def.linear_damping,
            angular_damping: def.angular_damping,
            gravity_scale: def.gravity_scale,
            mass: 0.0,
            inv_mass: 0.0,
            local_center: Vec3::ZERO,
            local_inertia: Mat33::ZERO,
            inv_local_inertia: Mat33::ZERO,
            inv_world_inertia: Mat33::ZERO,
            shapes: Vec::new(),
            is_awake: def.is_awake,
            enable_sleep: def.enable_sleep,
            sleep_time: 0.0,
        }
    }

    #[inline]
    pub fn get_transform(&self) -> Transform {
        Transform::new(self.position, self.rotation)
    }

    #[inline]
    pub fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    #[inline]
    pub fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    #[inline]
    pub fn is_kinematic(&self) -> bool {
        self.body_type == BodyType::Kinematic
    }

    pub fn set_mass_data(&mut self, mass_data: &MassData) {
        if self.body_type == BodyType::Static || self.body_type == BodyType::Kinematic {
            self.mass = 0.0;
            self.inv_mass = 0.0;
            self.local_center = Vec3::ZERO;
            self.local_inertia = Mat33::ZERO;
            self.inv_local_inertia = Mat33::ZERO;
            self.inv_world_inertia = Mat33::ZERO;
            return;
        }

        self.mass = mass_data.mass;
        self.inv_mass = mass_data.inv_mass;
        self.local_center = mass_data.center;
        self.local_inertia = mass_data.inertia;
        self.inv_local_inertia = mass_data.inv_inertia;
        self.update_world_inertia();
    }

    #[inline]
    pub fn update_world_inertia(&mut self) {
        if self.inv_mass > 0.0 {
            self.inv_world_inertia = Mat33::rotate_inertia(self.rotation, self.inv_local_inertia);
        } else {
            self.inv_world_inertia = Mat33::ZERO;
        }
    }

    #[inline]
    pub fn apply_force(&mut self, force: Vec3) {
        if self.is_dynamic() {
            self.force += force;
            self.is_awake = true;
        }
    }

    #[inline]
    pub fn apply_torque(&mut self, torque: Vec3) {
        if self.is_dynamic() {
            self.torque += torque;
            self.is_awake = true;
        }
    }

    #[inline]
    pub fn apply_force_at_point(&mut self, force: Vec3, point: Vec3) {
        if self.is_dynamic() {
            self.force += force;
            let r = point - self.position;
            self.torque += r.cross(force);
            self.is_awake = true;
        }
    }

    #[inline]
    pub fn apply_linear_impulse(&mut self, impulse: Vec3) {
        if self.is_dynamic() {
            self.linear_velocity += impulse * self.inv_mass;
            self.is_awake = true;
        }
    }

    #[inline]
    pub fn apply_angular_impulse(&mut self, impulse: Vec3) {
        if self.is_dynamic() {
            self.angular_velocity += self.inv_world_inertia * impulse;
            self.is_awake = true;
        }
    }
}
