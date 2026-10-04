// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Soft Step contact constraint solver.
//!
//! Sub-stepping constraint solver with compliance, spring-damper formulation,
//! and 2-axis Coulomb friction cones.

use crate::collision::manifold::ContactManifold;
use crate::dynamics::body::RigidBody;
use crate::math::{Mat22, Softness, Vec2, Vec3};

#[derive(Clone, Debug)]
pub struct SolverContactPoint {
    pub r_a: Vec3,
    pub r_b: Vec3,
    pub base_separation: f32,
    pub normal_mass: f32,
    pub normal_impulse: f32,
}

#[derive(Clone, Debug)]
pub struct SolverContact {
    pub body_a: usize,
    pub body_b: usize,
    pub normal: Vec3,
    pub tangent1: Vec3,
    pub tangent2: Vec3,
    pub center_a: Vec3,
    pub center_b: Vec3,
    pub tangent_mass: Mat22,
    pub friction_impulse: Vec2,
    pub friction: f32,
    pub restitution: f32,
    pub is_static: bool,
    pub points: Vec<SolverContactPoint>,
}

impl SolverContact {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        body_idx_a: usize,
        body_idx_b: usize,
        body_a: &RigidBody,
        body_b: &RigidBody,
        manifold: &ContactManifold,
        friction: f32,
        restitution: f32,
        warm_starting: bool,
    ) -> Self {
        let normal = manifold.normal;
        let tangent1 = normal.perp();
        let tangent2 = normal.cross(tangent1);

        let m_a = body_a.inv_mass;
        let m_b = body_b.inv_mass;
        let i_a = body_a.inv_world_inertia;
        let i_b = body_b.inv_world_inertia;

        let mut center_a = Vec3::ZERO;
        let mut center_b = Vec3::ZERO;
        let point_count = manifold.points.len() as f32;

        let mut solver_points = Vec::with_capacity(manifold.points.len());

        for mp in &manifold.points {
            let r_a = mp.anchor_a;
            let r_b = mp.anchor_b;
            center_a += r_a;
            center_b += r_b;

            let rn_a = r_a.cross(normal);
            let rn_b = r_b.cross(normal);
            let k_normal = m_a + m_b + rn_a.dot(i_a * rn_a) + rn_b.dot(i_b * rn_b);
            let normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            let s = mp.separation;
            let base_separation = s - (r_b - r_a).dot(normal);
            let normal_impulse = if warm_starting { mp.normal_impulse } else { 0.0 };

            solver_points.push(SolverContactPoint {
                r_a,
                r_b,
                base_separation,
                normal_mass,
                normal_impulse,
            });
        }

        if point_count > 0.0 {
            center_a *= 1.0 / point_count;
            center_b *= 1.0 / point_count;
        }

        // Tangent mass matrix
        let rt_a1 = center_a.cross(tangent1);
        let rt_a2 = center_a.cross(tangent2);
        let rt_b1 = center_b.cross(tangent1);
        let rt_b2 = center_b.cross(tangent2);

        let k11 = m_a + m_b + rt_a1.dot(i_a * rt_a1) + rt_b1.dot(i_b * rt_b1);
        let k22 = m_a + m_b + rt_a2.dot(i_a * rt_a2) + rt_b2.dot(i_b * rt_b2);
        let k12 = rt_a1.dot(i_a * rt_a2) + rt_b1.dot(i_b * rt_b2);

        let k_mat = Mat22::from_cols(Vec2::new(k11, k12), Vec2::new(k12, k22));
        let tangent_mass = k_mat.invert();

        let is_static = body_a.is_static() || body_b.is_static();

        Self {
            body_a: body_idx_a,
            body_b: body_idx_b,
            normal,
            tangent1,
            tangent2,
            center_a,
            center_b,
            tangent_mass,
            friction_impulse: Vec2::ZERO,
            friction,
            restitution,
            is_static,
            points: solver_points,
        }
    }
}

pub struct SolverContext<'a> {
    pub bodies: &'a mut [RigidBody],
    pub contacts: &'a mut [SolverContact],
    pub h: f32,
    pub inv_h: f32,
    pub contact_softness: Softness,
    pub static_softness: Softness,
    pub contact_speed: f32,
    pub max_linear_speed: f32,
}

/// Perform one sub-step of the Soft Step constraint solver
pub fn solve_sub_step(ctx: &mut SolverContext<'_>) {
    let inv_h = ctx.inv_h;
    let contact_speed = ctx.contact_speed;

    // 1. Solve contact constraints (Normal + Friction)
    for contact in ctx.contacts.iter_mut() {
        let softness = if contact.is_static {
            ctx.static_softness
        } else {
            ctx.contact_softness
        };

        let normal = contact.normal;
        let tangent1 = contact.tangent1;
        let tangent2 = contact.tangent2;
        let friction = contact.friction;

        let b_a = &ctx.bodies[contact.body_a];
        let b_b = &ctx.bodies[contact.body_b];

        let mut v_a = b_a.linear_velocity;
        let mut w_a = b_a.angular_velocity;
        let dp_a = b_a.delta_position;
        let dq_a = b_a.delta_rotation;
        let m_a = b_a.inv_mass;
        let i_a = b_a.inv_world_inertia;

        let mut v_b = b_b.linear_velocity;
        let mut w_b = b_b.angular_velocity;
        let dp_b = b_b.delta_position;
        let dq_b = b_b.delta_rotation;
        let m_b = b_b.inv_mass;
        let i_b = b_b.inv_world_inertia;

        let mut total_normal_impulse = 0.0f32;

        // Normal constraints
        for cp in &mut contact.points {
            let r_a = cp.r_a;
            let r_b = cp.r_b;

            // Separation displacement during sub-step
            let ds = (dp_b + dq_b.rotate_vec3(r_b)) - (dp_a + dq_a.rotate_vec3(r_a));
            let s = ds.dot(normal) + cp.base_separation;

            let (velocity_bias, mass_scale, impulse_scale) = if s > 0.0 {
                (s * inv_h, 1.0, 0.0)
            } else {
                let bias = (softness.mass_scale * softness.bias_rate * s).max(-contact_speed);
                (bias, softness.mass_scale, softness.impulse_scale)
            };

            let vr_a = v_a + w_a.cross(r_a);
            let vr_b = v_b + w_b.cross(r_b);
            let vn = (vr_b - vr_a).dot(normal);

            let delta_impulse =
                -cp.normal_mass * (mass_scale * vn + velocity_bias) - impulse_scale * cp.normal_impulse;
            let new_impulse = (cp.normal_impulse + delta_impulse).max(0.0);
            let delta_impulse = new_impulse - cp.normal_impulse;
            cp.normal_impulse = new_impulse;
            total_normal_impulse += new_impulse;

            // Apply normal impulse
            let p = normal * delta_impulse;
            v_a -= p * m_a;
            w_a -= i_a * r_a.cross(p);
            v_b += p * m_b;
            w_b += i_b * r_b.cross(p);
        }

        // Friction constraint (2-axis Coulomb friction cone)
        if total_normal_impulse > 0.0 && friction > 0.0 {
            let r_a = contact.center_a;
            let r_b = contact.center_b;

            let vr_a = v_a + w_a.cross(r_a);
            let vr_b = v_b + w_b.cross(r_b);
            let vr = vr_b - vr_a;
            let vt = Vec2::new(vr.dot(tangent1), vr.dot(tangent2));

            let delta_friction = -contact.tangent_mass.mul_vec(vt);
            let mut new_friction = contact.friction_impulse + delta_friction;
            let max_impulse = friction * total_normal_impulse;

            let len_sq = new_friction.length_squared();
            if len_sq > max_impulse * max_impulse {
                let scale = max_impulse / len_sq.sqrt();
                new_friction = new_friction * scale;
            }
            let delta_friction = new_friction - contact.friction_impulse;
            contact.friction_impulse = new_friction;

            let p_f = tangent1 * delta_friction.x + tangent2 * delta_friction.y;
            v_a -= p_f * m_a;
            w_a -= i_a * r_a.cross(p_f);
            v_b += p_f * m_b;
            w_b += i_b * r_b.cross(p_f);
        }

        // Write back velocities
        if ctx.bodies[contact.body_a].is_dynamic() {
            ctx.bodies[contact.body_a].linear_velocity = v_a;
            ctx.bodies[contact.body_a].angular_velocity = w_a;
        }
        if ctx.bodies[contact.body_b].is_dynamic() {
            ctx.bodies[contact.body_b].linear_velocity = v_b;
            ctx.bodies[contact.body_b].angular_velocity = w_b;
        }
    }

    // 2. Integrate positions for each dynamic body
    let h = ctx.h;
    for body in ctx.bodies.iter_mut() {
        if !body.is_dynamic() || !body.is_awake {
            continue;
        }

        let v = body.linear_velocity;
        let w = body.angular_velocity;

        body.delta_position += v * h;
        body.delta_rotation = body.delta_rotation.integrate(w * h);
    }
}
