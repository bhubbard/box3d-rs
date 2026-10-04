// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Math primitives and operations for 3D physics simulation.
//!
//! Directly mirrors Erin Catto's Box3D math formulations with Rust ergonomic operators.

use std::f32::consts::PI;
use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

pub const B3_PI: f32 = PI;
pub const DEG_TO_RAD: f32 = PI / 180.0;
pub const RAD_TO_DEG: f32 = 180.0 / PI;
pub const MIN_SCALE: f32 = 0.01;
pub const LINEAR_SLOP: f32 = 0.005;
pub const ANGULAR_SLOP: f32 = (2.0 / 180.0) * PI;
pub const SPECULATIVE_DISTANCE: f32 = 4.0 * LINEAR_SLOP;
pub const MAX_LINEAR_SPEED: f32 = 400.0;
pub const MAX_ANGULAR_SPEED: f32 = 50.0;

/// 2D Vector
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    #[inline]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Neg for Vec2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Mul<Vec2> for f32 {
    type Output = Vec2;
    #[inline]
    fn mul(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self * rhs.x, self * rhs.y)
    }
}

/// 3D Vector
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    pub const ONE: Self = Self { x: 1.0, y: 1.0, z: 1.0 };
    pub const X: Self = Self { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Self = Self { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Self = Self { x: 0.0, y: 0.0, z: 1.0 };

    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    #[inline]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    #[inline]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > 100.0 * f32::MIN_POSITIVE {
            let inv = 1.0 / len;
            Self {
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
            }
        } else {
            Self::ZERO
        }
    }

    #[inline]
    pub fn abs(self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
        }
    }

    #[inline]
    pub fn min(self, other: Self) -> Self {
        Self {
            x: self.x.min(other.x),
            y: self.y.min(other.y),
            z: self.z.min(other.z),
        }
    }

    #[inline]
    pub fn max(self, other: Self) -> Self {
        Self {
            x: self.x.max(other.x),
            y: self.y.max(other.y),
            z: self.z.max(other.z),
        }
    }

    #[inline]
    pub fn clamp(self, min_val: Self, max_val: Self) -> Self {
        Self {
            x: self.x.clamp(min_val.x, max_val.x),
            y: self.y.clamp(min_val.y, max_val.y),
            z: self.z.clamp(min_val.z, max_val.z),
        }
    }

    #[inline]
    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self {
            x: (1.0 - t) * self.x + t * other.x,
            y: (1.0 - t) * self.y + t * other.y,
            z: (1.0 - t) * self.z + t * other.z,
        }
    }

    /// Get a unit vector that is perpendicular to the supplied vector.
    #[inline]
    pub fn perp(self) -> Self {
        let p = if self.x < -0.5 || self.x > 0.5 {
            Self::new(self.y, -self.x, 0.0)
        } else {
            Self::new(0.0, self.z, -self.y)
        };
        p.normalize()
    }

    #[inline]
    pub fn is_valid(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Neg for Vec3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl MulAssign<f32> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Mul<Vec3> for f32 {
    type Output = Vec3;
    #[inline]
    fn mul(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

impl Mul<Vec3> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Vec3) -> Self {
        Self::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

/// Unit Quaternion representing 3D rotation
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Quat {
    pub v: Vec3,
    pub s: f32,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Self = Self {
        v: Vec3::ZERO,
        s: 1.0,
    };

    #[inline]
    pub const fn new(x: f32, y: f32, z: f32, s: f32) -> Self {
        Self {
            v: Vec3::new(x, y, z),
            s,
        }
    }

    #[inline]
    pub const fn from_vec3_scalar(v: Vec3, s: f32) -> Self {
        Self { v, s }
    }

    #[inline]
    pub fn from_axis_angle(axis: Vec3, radians: f32) -> Self {
        let half_angle = 0.5 * radians;
        let sin_half = half_angle.sin();
        let cos_half = half_angle.cos();
        Self {
            v: axis * sin_half,
            s: cos_half,
        }
    }

    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.v.dot(other.v) + self.s * other.s
    }

    #[inline]
    pub fn conjugate(self) -> Self {
        Self {
            v: -self.v,
            s: self.s,
        }
    }

    #[inline]
    pub fn normalize(self) -> Self {
        let len_sq = self.dot(self);
        if len_sq > 1000.0 * f32::MIN_POSITIVE {
            let inv = 1.0 / len_sq.sqrt();
            Self {
                v: self.v * inv,
                s: self.s * inv,
            }
        } else {
            Self::IDENTITY
        }
    }

    /// Multiply two quaternions (q1 * q2)
    #[inline]
    pub fn mul_quat(self, other: Self) -> Self {
        let t1 = self.v.cross(other.v);
        let t2 = t1 + other.v * self.s;
        let t3 = t2 + self.v * other.s;
        Self {
            v: t3,
            s: self.s * other.s - self.v.dot(other.v),
        }
    }

    /// Relative quaternion: inv(self) * other
    #[inline]
    pub fn inv_mul_quat(self, other: Self) -> Self {
        self.conjugate().mul_quat(other)
    }

    /// Rotate a vector by this quaternion
    #[inline]
    pub fn rotate_vec3(self, v: Vec3) -> Vec3 {
        let t1 = self.v.cross(v);
        let t2 = t1 + v * self.s;
        let t3 = self.v.cross(t2);
        v + t3 * 2.0
    }

    /// Inverse rotate a vector by this quaternion
    #[inline]
    pub fn inv_rotate_vec3(self, v: Vec3) -> Vec3 {
        let t1 = self.v.cross(v);
        let t2 = t1 - v * self.s;
        let t3 = self.v.cross(t2);
        v + t3 * 2.0
    }

    /// Get axis and angle in radians
    #[inline]
    pub fn to_axis_angle(self) -> (Vec3, f32) {
        let len = self.v.length();
        let angle = 2.0 * len.atan2(self.s);
        let axis = if len > 0.0 {
            self.v * (1.0 / len)
        } else {
            Vec3::ZERO
        };
        (axis, angle)
    }

    /// Integrate rotation with angular velocity displacement
    #[inline]
    pub fn integrate(self, delta_rotation: Vec3) -> Self {
        let qd = Quat {
            v: delta_rotation * 0.5,
            s: 0.0,
        };
        let qd = qd.mul_quat(self);
        let q2 = Quat {
            v: self.v + qd.v,
            s: self.s + qd.s,
        };
        q2.normalize()
    }
}

impl Mul<Quat> for Quat {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        self.mul_quat(rhs)
    }
}

/// 3x3 Matrix stored as column vectors
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Mat33 {
    pub cx: Vec3,
    pub cy: Vec3,
    pub cz: Vec3,
}

impl Default for Mat33 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mat33 {
    pub const ZERO: Self = Self {
        cx: Vec3::ZERO,
        cy: Vec3::ZERO,
        cz: Vec3::ZERO,
    };

    pub const IDENTITY: Self = Self {
        cx: Vec3::X,
        cy: Vec3::Y,
        cz: Vec3::Z,
    };

    #[inline]
    pub const fn from_cols(cx: Vec3, cy: Vec3, cz: Vec3) -> Self {
        Self { cx, cy, cz }
    }

    #[inline]
    pub fn from_diagonal(dx: f32, dy: f32, dz: f32) -> Self {
        Self {
            cx: Vec3::new(dx, 0.0, 0.0),
            cy: Vec3::new(0.0, dy, 0.0),
            cz: Vec3::new(0.0, 0.0, dz),
        }
    }

    /// Create 3x3 rotation matrix from quaternion
    #[inline]
    pub fn from_quat(q: Quat) -> Self {
        let qx2 = q.v.x + q.v.x;
        let qy2 = q.v.y + q.v.y;
        let qz2 = q.v.z + q.v.z;

        let xx = q.v.x * qx2;
        let yy = q.v.y * qy2;
        let zz = q.v.z * qz2;
        let xy = q.v.x * qy2;
        let xz = q.v.x * qz2;
        let yz = q.v.y * qz2;
        let wx = q.s * qx2;
        let wy = q.s * qy2;
        let wz = q.s * qz2;

        Self {
            cx: Vec3::new(1.0 - yy - zz, xy + wz, xz - wy),
            cy: Vec3::new(xy - wz, 1.0 - xx - zz, yz + wx),
            cz: Vec3::new(xz + wy, yz - wx, 1.0 - xx - yy),
        }
    }

    #[inline]
    pub fn transpose(self) -> Self {
        Self {
            cx: Vec3::new(self.cx.x, self.cy.x, self.cz.x),
            cy: Vec3::new(self.cx.y, self.cy.y, self.cz.y),
            cz: Vec3::new(self.cx.z, self.cy.z, self.cz.z),
        }
    }

    #[inline]
    pub fn mul_vec(self, v: Vec3) -> Vec3 {
        self.cx * v.x + self.cy * v.y + self.cz * v.z
    }

    #[inline]
    pub fn mul_mat(self, b: Self) -> Self {
        Self {
            cx: self.mul_vec(b.cx),
            cy: self.mul_vec(b.cy),
            cz: self.mul_vec(b.cz),
        }
    }

    #[inline]
    pub fn invert(self) -> Self {
        let c = self.cy.cross(self.cz);
        let det = self.cx.dot(c);
        if det.abs() > 1000.0 * f32::MIN_POSITIVE {
            let inv_det = 1.0 / det;
            let a = self.cx;
            let b = self.cy;
            let d = self.cz;

            let row0 = b.cross(d);
            let row1 = d.cross(a);
            let row2 = a.cross(b);

            Self {
                cx: Vec3::new(row0.x * inv_det, row1.x * inv_det, row2.x * inv_det),
                cy: Vec3::new(row0.y * inv_det, row1.y * inv_det, row2.y * inv_det),
                cz: Vec3::new(row0.z * inv_det, row1.z * inv_det, row2.z * inv_det),
            }
        } else {
            Self::ZERO
        }
    }

    #[inline]
    pub fn solve(self, b: Vec3) -> Vec3 {
        self.invert().mul_vec(b)
    }

    /// Inertia tensor of a box with given mass and half-extents
    #[inline]
    pub fn box_inertia(mass: f32, h: Vec3) -> Self {
        let size = h * 2.0;
        let ixx = mass * (size.y * size.y + size.z * size.z) / 12.0;
        let iyy = mass * (size.x * size.x + size.z * size.z) / 12.0;
        let izz = mass * (size.x * size.x + size.y * size.y) / 12.0;
        Self::from_diagonal(ixx, iyy, izz)
    }

    /// Parallel axis theorem: shift inertia tensor to/from center of mass
    #[inline]
    pub fn steiner(mass: f32, origin: Vec3) -> Self {
        let ixx = mass * (origin.y * origin.y + origin.z * origin.z);
        let iyy = mass * (origin.x * origin.x + origin.z * origin.z);
        let izz = mass * (origin.x * origin.x + origin.y * origin.y);
        let ixy = -mass * origin.x * origin.y;
        let ixz = -mass * origin.x * origin.z;
        let iyz = -mass * origin.y * origin.z;

        Self {
            cx: Vec3::new(ixx, ixy, ixz),
            cy: Vec3::new(ixy, iyy, iyz),
            cz: Vec3::new(ixz, iyz, izz),
        }
    }

    /// Transform an inertia tensor by a rotation quaternion: R * I * R^T
    #[inline]
    pub fn rotate_inertia(q: Quat, inertia: Self) -> Self {
        let r = Self::from_quat(q);
        r.mul_mat(inertia.mul_mat(r.transpose()))
    }
}

impl Add for Mat33 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            cx: self.cx + rhs.cx,
            cy: self.cy + rhs.cy,
            cz: self.cz + rhs.cz,
        }
    }
}

impl Mul<Vec3> for Mat33 {
    type Output = Vec3;
    #[inline]
    fn mul(self, rhs: Vec3) -> Vec3 {
        self.mul_vec(rhs)
    }
}

impl Mul<Mat33> for Mat33 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        self.mul_mat(rhs)
    }
}

impl Mul<f32> for Mat33 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self {
            cx: self.cx * rhs,
            cy: self.cy * rhs,
            cz: self.cz * rhs,
        }
    }
}

/// 2x2 Matrix for tangent friction
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Mat22 {
    pub cx: Vec2,
    pub cy: Vec2,
}

impl Mat22 {
    #[inline]
    pub const fn from_cols(cx: Vec2, cy: Vec2) -> Self {
        Self { cx, cy }
    }

    #[inline]
    pub fn mul_vec(self, v: Vec2) -> Vec2 {
        self.cx * v.x + self.cy * v.y
    }

    #[inline]
    pub fn invert(self) -> Self {
        let a = self.cx.x;
        let b = self.cy.x;
        let c = self.cx.y;
        let d = self.cy.y;
        let det = a * d - b * c;
        if det.abs() > 1000.0 * f32::MIN_POSITIVE {
            let inv_det = 1.0 / det;
            Self {
                cx: Vec2::new(d * inv_det, -c * inv_det),
                cy: Vec2::new(-b * inv_det, a * inv_det),
            }
        } else {
            Self::default()
        }
    }

    #[inline]
    pub fn solve(self, b: Vec2) -> Vec2 {
        self.invert().mul_vec(b)
    }
}

impl Mul<Vec2> for Mat22 {
    type Output = Vec2;
    #[inline]
    fn mul(self, rhs: Vec2) -> Vec2 {
        self.mul_vec(rhs)
    }
}

impl Mul<Mat22> for Mat22 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            cx: self.mul_vec(rhs.cx),
            cy: self.mul_vec(rhs.cy),
        }
    }
}

/// Rigid transform (Translation and Rotation)
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Transform {
    pub p: Vec3,
    pub q: Quat,
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform {
    pub const IDENTITY: Self = Self {
        p: Vec3::ZERO,
        q: Quat::IDENTITY,
    };

    #[inline]
    pub const fn new(p: Vec3, q: Quat) -> Self {
        Self { p, q }
    }

    #[inline]
    pub const fn from_translation(p: Vec3) -> Self {
        Self {
            p,
            q: Quat::IDENTITY,
        }
    }

    #[inline]
    pub const fn from_rotation(q: Quat) -> Self {
        Self { p: Vec3::ZERO, q }
    }

    /// Multiply two transforms: a * b
    #[inline]
    pub fn mul_transforms(a: Self, b: Self) -> Self {
        Self {
            p: a.q.rotate_vec3(b.p) + a.p,
            q: a.q.mul_quat(b.q),
        }
    }
}

impl std::ops::Mul for Transform {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self::mul_transforms(self, rhs)
    }
}

impl Transform {

    /// Inverse multiply: inv(a) * b
    #[inline]
    pub fn inv_mul_transforms(a: Self, b: Self) -> Self {
        Self {
            p: a.q.inv_rotate_vec3(b.p - a.p),
            q: a.q.inv_mul_quat(b.q),
        }
    }

    #[inline]
    pub fn inv_mul_transform(self, b: Self) -> Self {
        Self::inv_mul_transforms(self, b)
    }

    /// Invert a transform
    #[inline]
    pub fn invert(self) -> Self {
        Self {
            p: self.q.inv_rotate_vec3(-self.p),
            q: self.q.conjugate(),
        }
    }

    /// Transform a point: q * v + p
    #[inline]
    pub fn transform_point(self, v: Vec3) -> Vec3 {
        self.q.rotate_vec3(v) + self.p
    }

    /// Inverse transform a point: inv(q) * (v - p)
    #[inline]
    pub fn inv_transform_point(self, v: Vec3) -> Vec3 {
        self.q.inv_rotate_vec3(v - self.p)
    }
}

/// Axis-Aligned Bounding Box (AABB)
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AABB {
    pub lower_bound: Vec3,
    pub upper_bound: Vec3,
}

impl Default for AABB {
    fn default() -> Self {
        Self {
            lower_bound: Vec3::new(f32::MAX, f32::MAX, f32::MAX),
            upper_bound: Vec3::new(f32::MIN, f32::MIN, f32::MIN),
        }
    }
}

impl AABB {
    #[inline]
    pub const fn new(lower_bound: Vec3, upper_bound: Vec3) -> Self {
        Self {
            lower_bound,
            upper_bound,
        }
    }

    #[inline]
    pub fn center(self) -> Vec3 {
        (self.lower_bound + self.upper_bound) * 0.5
    }

    #[inline]
    pub fn extents(self) -> Vec3 {
        self.upper_bound - self.lower_bound
    }

    #[inline]
    pub fn half_extents(self) -> Vec3 {
        self.extents() * 0.5
    }

    #[inline]
    pub fn surface_area(self) -> f32 {
        let d = self.extents();
        2.0 * (d.x * d.y + d.x * d.z + d.y * d.z)
    }

    #[inline]
    pub fn contains(self, other: Self) -> bool {
        self.lower_bound.x <= other.lower_bound.x
            && self.lower_bound.y <= other.lower_bound.y
            && self.lower_bound.z <= other.lower_bound.z
            && other.upper_bound.x <= self.upper_bound.x
            && other.upper_bound.y <= self.upper_bound.y
            && other.upper_bound.z <= self.upper_bound.z
    }

    #[inline]
    pub fn contains_point(self, p: Vec3) -> bool {
        self.lower_bound.x <= p.x && p.x <= self.upper_bound.x
            && self.lower_bound.y <= p.y && p.y <= self.upper_bound.y
            && self.lower_bound.z <= p.z && p.z <= self.upper_bound.z
    }

    #[inline]
    pub fn overlaps(self, other: Self) -> bool {
        !(self.lower_bound.x > other.upper_bound.x
            || self.lower_bound.y > other.upper_bound.y
            || self.lower_bound.z > other.upper_bound.z
            || other.lower_bound.x > self.upper_bound.x
            || other.lower_bound.y > self.upper_bound.y
            || other.lower_bound.z > self.upper_bound.z)
    }

    #[inline]
    pub fn union(self, other: Self) -> Self {
        Self {
            lower_bound: self.lower_bound.min(other.lower_bound),
            upper_bound: self.upper_bound.max(other.upper_bound),
        }
    }

    #[inline]
    pub fn add_point(self, p: Vec3) -> Self {
        Self {
            lower_bound: self.lower_bound.min(p),
            upper_bound: self.upper_bound.max(p),
        }
    }

    /// Transform an AABB by a rigid transform
    #[inline]
    pub fn transform(self, xf: Transform) -> Self {
        let center = self.center();
        let h = self.half_extents();
        let r = Mat33::from_quat(xf.q);

        // Absolute rotation matrix
        let ar = Mat33 {
            cx: r.cx.abs(),
            cy: r.cy.abs(),
            cz: r.cz.abs(),
        };

        let world_center = xf.transform_point(center);
        let world_h = ar.mul_vec(h);

        Self {
            lower_bound: world_center - world_h,
            upper_bound: world_center + world_h,
        }
    }

    /// Raycast against AABB
    #[inline]
    pub fn ray_cast(self, origin: Vec3, direction: Vec3, max_t: f32) -> Option<f32> {
        let mut t_min = 0.0f32;
        let mut t_max = max_t;

        for i in 0..3 {
            let (o, d, min_b, max_b) = match i {
                0 => (origin.x, direction.x, self.lower_bound.x, self.upper_bound.x),
                1 => (origin.y, direction.y, self.lower_bound.y, self.upper_bound.y),
                _ => (origin.z, direction.z, self.lower_bound.z, self.upper_bound.z),
            };

            if d.abs() < 1e-8 {
                if o < min_b || o > max_b {
                    return None;
                }
            } else {
                let inv_d = 1.0 / d;
                let mut t1 = (min_b - o) * inv_d;
                let mut t2 = (max_b - o) * inv_d;
                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }
                t_min = t_min.max(t1);
                t_max = t_max.min(t2);
                if t_min > t_max {
                    return None;
                }
            }
        }

        Some(t_min)
    }
}

/// A 3D Plane: separation = dot(normal, point) - offset
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Plane {
    pub normal: Vec3,
    pub offset: f32,
}

impl Plane {
    #[inline]
    pub const fn new(normal: Vec3, offset: f32) -> Self {
        Self { normal, offset }
    }

    #[inline]
    pub fn from_normal_and_point(normal: Vec3, point: Vec3) -> Self {
        Self {
            normal,
            offset: normal.dot(point),
        }
    }

    #[inline]
    pub fn separation(self, point: Vec3) -> f32 {
        self.normal.dot(point) - self.offset
    }

    #[inline]
    pub fn transform(self, xf: Transform) -> Self {
        let normal = xf.q.rotate_vec3(self.normal);
        let point = self.normal * self.offset;
        let world_point = xf.transform_point(point);
        Self {
            normal,
            offset: normal.dot(world_point),
        }
    }
}

/// Soft constraint parameters for sub-stepping
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Softness {
    pub bias_rate: f32,
    pub mass_scale: f32,
    pub impulse_scale: f32,
}

impl Softness {
    #[inline]
    pub fn new(hertz: f32, zeta: f32, h: f32) -> Self {
        if hertz == 0.0 {
            return Self {
                bias_rate: 0.0,
                mass_scale: 0.0,
                impulse_scale: 0.0,
            };
        }

        let omega = 2.0 * PI * hertz;
        let a1 = 2.0 * zeta + h * omega;
        let a2 = h * omega * a1;
        let a3 = 1.0 / (1.0 + a2);

        Self {
            bias_rate: omega / a1,
            mass_scale: a2 * a3,
            impulse_scale: a3,
        }
    }
}
