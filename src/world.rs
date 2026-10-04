// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Physics World simulation manager.
//!
//! Coordinates the broadphase dynamic tree, narrowphase SAT collision,
//! island management, and Soft Step constraint solver sub-stepping.

use crate::collision::broad_phase::BroadPhase;
use crate::collision::hull::ConvexHull;
use crate::collision::manifold::collide_shapes;
use crate::collision::shape::{Capsule, Shape, ShapeGeometry, Sphere};
use crate::dynamics::body::RigidBody;
use crate::dynamics::joints::{Joint, JointId, JointKind};
use crate::dynamics::solver::{solve_sub_step, SolverContact, SolverContext};
use crate::id::{BodyId, IdPool, ShapeId, WorldId};
use crate::math::{Mat33, Quat, Softness, Vec3};
use crate::types::{BodyDef, MassData, ShapeDef, WorldDef};

/// The primary simulation hub managing memory, bodies, shapes, and simulation steps.
pub struct World {
    pub id: WorldId,
    pub def: WorldDef,
    bodies: Vec<Option<RigidBody>>,
    shapes: Vec<Option<Shape>>,
    joints: Vec<Option<Joint>>,
    body_pool: IdPool,
    shape_pool: IdPool,
    joint_pool: IdPool,
    broad_phase: BroadPhase,
    step_index: u64,
}

impl World {
    /// Create a new physics simulation world
    pub fn new(def: WorldDef) -> Self {
        Self {
            id: WorldId {
                index1: 1,
                generation: 1,
            },
            def,
            bodies: Vec::new(),
            shapes: Vec::new(),
            joints: Vec::new(),
            body_pool: IdPool::new(),
            shape_pool: IdPool::new(),
            joint_pool: IdPool::new(),
            broad_phase: BroadPhase::new(),
            step_index: 0,
        }
    }

    /// Check if world is valid
    pub fn is_valid(&self) -> bool {
        self.id.is_valid()
    }

    /// Create a rigid body in this world
    pub fn create_body(&mut self, def: &BodyDef) -> BodyId {
        let (idx, gen) = self.body_pool.allocate();
        let body_id = BodyId {
            index1: (idx + 1) as i32,
            world0: self.id.index1,
            generation: gen,
        };

        let mut body = RigidBody::new(body_id, def);
        body.update_world_inertia();

        if idx >= self.bodies.len() {
            self.bodies.resize_with(idx + 1, || None);
        }
        self.bodies[idx] = Some(body);

        body_id
    }

    /// Destroy a rigid body and its attached shapes and joints
    pub fn destroy_body(&mut self, body_id: BodyId) {
        let idx = (body_id.index1 - 1) as usize;
        if !self.body_pool.is_valid(idx, body_id.generation) {
            return;
        }

        if let Some(body) = self.bodies[idx].take() {
            for shape_id in body.shapes {
                self.destroy_shape(shape_id);
            }
            self.body_pool.free(idx);
        }
    }

    /// Create a polyhedral convex hull shape on a body
    pub fn create_hull_shape(&mut self, body_id: BodyId, def: &ShapeDef, hull: &ConvexHull) -> ShapeId {
        self.create_shape_internal(body_id, def, ShapeGeometry::Hull(hull.clone()))
    }

    /// Create a sphere shape on a body
    pub fn create_sphere_shape(&mut self, body_id: BodyId, def: &ShapeDef, sphere: Sphere) -> ShapeId {
        self.create_shape_internal(body_id, def, ShapeGeometry::Sphere(sphere))
    }

    /// Create a capsule shape on a body
    pub fn create_capsule_shape(&mut self, body_id: BodyId, def: &ShapeDef, capsule: Capsule) -> ShapeId {
        self.create_shape_internal(body_id, def, ShapeGeometry::Capsule(capsule))
    }

    fn create_shape_internal(&mut self, body_id: BodyId, def: &ShapeDef, geom: ShapeGeometry) -> ShapeId {
        let (idx, gen) = self.shape_pool.allocate();
        let shape_id = ShapeId {
            index1: (idx + 1) as i32,
            world0: self.id.index1,
            generation: gen,
        };

        let mut shape = Shape::new(shape_id, body_id, geom, def);

        // Attach to body
        let b_idx = (body_id.index1 - 1) as usize;
        let is_dynamic = if let Some(body) = &mut self.bodies[b_idx] {
            body.shapes.push(shape_id);
            // Recompute mass data from shapes
            let mass_data = shape.compute_mass_data();
            body.set_mass_data(&mass_data);

            let xf = body.get_transform();
            let aabb = shape.geometry.compute_aabb(xf);
            shape.fat_aabb = aabb;
            body.is_dynamic()
        } else {
            false
        };

        // Add to broadphase
        let proxy_id = self.broad_phase.create_proxy(shape.fat_aabb, is_dynamic, idx as i32);
        shape.proxy_id = proxy_id;

        if idx >= self.shapes.len() {
            self.shapes.resize_with(idx + 1, || None);
        }
        self.shapes[idx] = Some(shape);

        shape_id
    }

    /// Destroy a shape
    pub fn destroy_shape(&mut self, shape_id: ShapeId) {
        let idx = (shape_id.index1 - 1) as usize;
        if !self.shape_pool.is_valid(idx, shape_id.generation) {
            return;
        }

        if let Some(shape) = self.shapes[idx].take() {
            let b_idx = (shape.body_id.index1 - 1) as usize;
            let is_dynamic = self.bodies.get(b_idx).and_then(|b| b.as_ref()).is_some_and(|b| b.is_dynamic());
            self.broad_phase.destroy_proxy(shape.proxy_id, is_dynamic);
            self.shape_pool.free(idx);

            if let Some(Some(body)) = self.bodies.get_mut(b_idx) {
                body.shapes.retain(|&s| s != shape_id);
                let mut total_mass = 0.0;
                let mut total_center = Vec3::ZERO;
                let mut total_inertia = Mat33::ZERO;
                for &s_id in &body.shapes {
                    let s_i = (s_id.index1 - 1) as usize;
                    if let Some(Some(s)) = self.shapes.get(s_i) {
                        let md = s.compute_mass_data();
                        total_mass += md.mass;
                        total_center += md.center * md.mass;
                        total_inertia = total_inertia + md.inertia;
                    }
                }
                if total_mass > 0.0 {
                    total_center *= 1.0 / total_mass;
                }
                body.set_mass_data(&MassData::new(total_mass, total_center, total_inertia));
            }
        }
    }

    /// Add a joint
    pub fn create_joint(&mut self, kind: JointKind) -> JointId {
        let (idx, gen) = self.joint_pool.allocate();
        let joint_id = JointId {
            index1: (idx + 1) as i32,
            world0: self.id.index1,
            generation: gen,
        };

        let joint = Joint::new(joint_id, kind);
        if idx >= self.joints.len() {
            self.joints.resize_with(idx + 1, || None);
        }
        self.joints[idx] = Some(joint);

        joint_id
    }

    /// Destroy a joint
    pub fn destroy_joint(&mut self, joint_id: JointId) {
        let idx = (joint_id.index1 - 1) as usize;
        if !self.joint_pool.is_valid(idx, joint_id.generation) {
            return;
        }
        self.joints[idx] = None;
        self.joint_pool.free(idx);
    }

    /// Get body reference
    pub fn get_body(&self, body_id: BodyId) -> Option<&RigidBody> {
        let idx = (body_id.index1 - 1) as usize;
        if self.body_pool.is_valid(idx, body_id.generation) {
            self.bodies.get(idx).and_then(|b| b.as_ref())
        } else {
            None
        }
    }

    /// Get mutable body reference
    pub fn get_body_mut(&mut self, body_id: BodyId) -> Option<&mut RigidBody> {
        let idx = (body_id.index1 - 1) as usize;
        if self.body_pool.is_valid(idx, body_id.generation) {
            self.bodies.get_mut(idx).and_then(|b| b.as_mut())
        } else {
            None
        }
    }

    /// Get body world position
    pub fn get_body_position(&self, body_id: BodyId) -> Vec3 {
        self.get_body(body_id).map_or(Vec3::ZERO, |b| b.position)
    }

    /// Get body world rotation quaternion
    pub fn get_body_rotation(&self, body_id: BodyId) -> Quat {
        self.get_body(body_id).map_or(Quat::IDENTITY, |b| b.rotation)
    }

    /// Step the simulation forward by `time_step` using `sub_step_count` sub-steps
    pub fn step(&mut self, time_step: f32, sub_step_count: i32) {
        if time_step <= 0.0 {
            return;
        }

        self.step_index += 1;
        let sub_steps = sub_step_count.max(1);
        let h = time_step / sub_steps as f32;
        let inv_h = 1.0 / h;

        // 1. Update broadphase proxies for moving bodies
        for body in self.bodies.iter_mut().flatten() {
            if !body.is_dynamic() {
                continue;
            }
            let xf = body.get_transform();
            let disp = body.linear_velocity * time_step;

            for &shape_id in &body.shapes {
                let s_idx = (shape_id.index1 - 1) as usize;
                if let Some(Some(shape)) = self.shapes.get_mut(s_idx) {
                    let aabb = shape.geometry.compute_aabb(xf);
                    shape.fat_aabb = aabb;
                    self.broad_phase.move_proxy(shape.proxy_id, aabb, disp);
                }
            }
        }

        // 2. Broadphase: find candidate pairs for all active dynamic shapes
        let mut candidate_pairs = Vec::new();
        for body in self.bodies.iter().flatten() {
            if !body.is_dynamic() || !body.is_awake {
                continue;
            }
            for &shape_id in &body.shapes {
                let s_idx = (shape_id.index1 - 1) as usize;
                if let Some(Some(shape)) = self.shapes.get(s_idx) {
                    let query_aabb = shape.fat_aabb;
                    let s_a = s_idx as i32;

                    self.broad_phase.dynamic_tree.query(query_aabb, |s_b| {
                        if s_a != s_b {
                            let pair = if s_a < s_b {
                                (s_a as usize, s_b as usize)
                            } else {
                                (s_b as usize, s_a as usize)
                            };
                            candidate_pairs.push(pair);
                        }
                        true
                    });

                    self.broad_phase.static_tree.query(query_aabb, |s_b| {
                        candidate_pairs.push((s_a as usize, s_b as usize));
                        true
                    });
                }
            }
        }

        candidate_pairs.sort_unstable();
        candidate_pairs.dedup();

        // 3. Narrowphase: generate contact manifolds
        let mut contacts = Vec::new();
        for (idx_a, idx_b) in candidate_pairs {
            let shape_a = match &self.shapes[idx_a] {
                Some(s) => s,
                None => continue,
            };
            let shape_b = match &self.shapes[idx_b] {
                Some(s) => s,
                None => continue,
            };

            if shape_a.body_id == shape_b.body_id {
                continue;
            }

            let b_idx_a = (shape_a.body_id.index1 - 1) as usize;
            let b_idx_b = (shape_b.body_id.index1 - 1) as usize;

            let body_a = match &self.bodies[b_idx_a] {
                Some(b) => b,
                None => continue,
            };
            let body_b = match &self.bodies[b_idx_b] {
                Some(b) => b,
                None => continue,
            };

            if body_a.is_static() && body_b.is_static() {
                continue;
            }

            let xf_a = body_a.get_transform();
            let xf_b = body_b.get_transform();

            if let Some(manifold) = collide_shapes(&shape_a.geometry, xf_a, &shape_b.geometry, xf_b) {
                if !manifold.is_empty() {
                    let friction = (shape_a.material.friction * shape_b.material.friction).sqrt();
                    let restitution = shape_a.material.restitution.max(shape_b.material.restitution);

                    let solver_contact = SolverContact::new(
                        b_idx_a,
                        b_idx_b,
                        body_a,
                        body_b,
                        &manifold,
                        friction,
                        restitution,
                        self.def.enable_warm_starting,
                    );
                    contacts.push(solver_contact);
                }
            }
        }

        // Wake up sleeping bodies that have active contacts
        for contact in &contacts {
            if let Some(Some(body)) = self.bodies.get_mut(contact.body_a) {
                if body.is_dynamic() && !body.is_awake {
                    body.is_awake = true;
                    body.sleep_time = 0.0;
                }
            }
            if let Some(Some(body)) = self.bodies.get_mut(contact.body_b) {
                if body.is_dynamic() && !body.is_awake {
                    body.is_awake = true;
                    body.sleep_time = 0.0;
                }
            }
        }

        // 4. Sub-stepping loop
        let contact_hertz = self.def.contact_hertz.min(0.125 * inv_h);
        let contact_softness = Softness::new(contact_hertz, self.def.contact_damping_ratio, h);
        let static_softness = Softness::new(2.0 * contact_hertz, 0.5 * self.def.contact_damping_ratio, h);

        // Flatten active bodies for mutable access
        let mut active_bodies: Vec<RigidBody> = self.bodies.iter().filter_map(|b| b.clone()).collect();
        let body_id_to_active_idx: Vec<Option<usize>> = self
            .bodies
            .iter()
            .enumerate()
            .map(|(i, b)| if b.is_some() { Some(i) } else { None })
            .collect();

        for _sub_step in 0..sub_steps {
            // Integrate velocities for active dynamic bodies
            let gravity = self.def.gravity;
            for body in active_bodies.iter_mut() {
                if !body.is_dynamic() || !body.is_awake {
                    continue;
                }

                let lin_damping = 1.0 / (1.0 + h * body.linear_damping);
                let ang_damping = 1.0 / (1.0 + h * body.angular_damping);

                let grav_scale = body.gravity_scale;
                let lin_delta = (body.force * body.inv_mass + gravity * grav_scale) * h;
                body.linear_velocity = (body.linear_velocity + lin_delta) * lin_damping;

                let ang_delta = (body.inv_world_inertia * body.torque) * h;
                body.angular_velocity = (body.angular_velocity + ang_delta) * ang_damping;
            }

            // Solve constraints
            let mut ctx = SolverContext {
                bodies: &mut active_bodies,
                contacts: &mut contacts,
                h,
                inv_h,
                contact_softness,
                static_softness,
                contact_speed: self.def.contact_speed,
                max_linear_speed: self.def.max_linear_speed,
            };
            solve_sub_step(&mut ctx);
        }

        // 5. Finalize body transforms
        for body in active_bodies.iter_mut() {
            if !body.is_dynamic() {
                continue;
            }

            body.position += body.delta_position;
            body.rotation = (body.delta_rotation * body.rotation).normalize();
            body.delta_position = Vec3::ZERO;
            body.delta_rotation = Quat::IDENTITY;
            body.update_world_inertia();

            // Clear forces
            body.force = Vec3::ZERO;
            body.torque = Vec3::ZERO;

            // Sleeping check
            if self.def.enable_sleep && body.enable_sleep {
                let lin_speed_sq = body.linear_velocity.length_squared();
                let ang_speed_sq = body.angular_velocity.length_squared();
                let sleep_linear_tol = 0.02; // 2 cm/s
                let sleep_angular_tol = 0.05; // ~3 deg/s

                if lin_speed_sq < sleep_linear_tol * sleep_linear_tol
                    && ang_speed_sq < sleep_angular_tol * sleep_angular_tol
                {
                    body.sleep_time += time_step;
                    if body.sleep_time > 0.5 {
                        body.is_awake = false;
                        body.linear_velocity = Vec3::ZERO;
                        body.angular_velocity = Vec3::ZERO;
                    }
                } else {
                    body.sleep_time = 0.0;
                }
            }
        }

        // Copy back to self.bodies
        for (i, active_idx) in body_id_to_active_idx.into_iter().enumerate() {
            if let Some(idx) = active_idx {
                self.bodies[i] = Some(active_bodies[idx].clone());
            }
        }
    }
}
