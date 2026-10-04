// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Broad-phase pair management.
//!
//! Uses separate dynamic trees for static vs dynamic bodies to avoid unnecessary static-static checks.

use crate::collision::dynamic_tree::DynamicTree;
use crate::math::{AABB, Vec3};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct ShapePair {
    pub shape_a: i32,
    pub shape_b: i32,
}

impl ShapePair {
    pub fn new(a: i32, b: i32) -> Self {
        if a < b {
            Self {
                shape_a: a,
                shape_b: b,
            }
        } else {
            Self {
                shape_a: b,
                shape_b: a,
            }
        }
    }
}

pub struct BroadPhase {
    pub static_tree: DynamicTree,
    pub dynamic_tree: DynamicTree,
    pub move_list: Vec<i32>,
}

impl Default for BroadPhase {
    fn default() -> Self {
        Self::new()
    }
}

impl BroadPhase {
    pub fn new() -> Self {
        Self {
            static_tree: DynamicTree::new(),
            dynamic_tree: DynamicTree::new(),
            move_list: Vec::new(),
        }
    }

    pub fn create_proxy(&mut self, aabb: AABB, is_dynamic: bool, shape_index: i32) -> i32 {
        if is_dynamic {
            let proxy_id = self.dynamic_tree.create_proxy(aabb, shape_index);
            self.move_list.push(proxy_id);
            proxy_id
        } else {
            self.static_tree.create_proxy(aabb, shape_index)
        }
    }

    pub fn destroy_proxy(&mut self, proxy_id: i32, is_dynamic: bool) {
        if is_dynamic {
            self.dynamic_tree.destroy_proxy(proxy_id);
            self.move_list.retain(|&id| id != proxy_id);
        } else {
            self.static_tree.destroy_proxy(proxy_id);
        }
    }

    pub fn move_proxy(&mut self, proxy_id: i32, aabb: AABB, displacement: Vec3) {
        let moved = self.dynamic_tree.move_proxy(proxy_id, aabb, displacement);
        if moved && !self.move_list.contains(&proxy_id) {
            self.move_list.push(proxy_id);
        }
    }

    /// Query potential pairs between moved dynamic shapes and all other shapes.
    pub fn find_pairs<F>(&mut self, mut callback: F)
    where
        F: FnMut(i32, i32),
    {
        for &query_proxy in &self.move_list {
            let fat_aabb = self.dynamic_tree.get_fat_aabb(query_proxy);
            let shape_index_a = self.dynamic_tree.get_user_data(query_proxy);

            // Query dynamic tree
            self.dynamic_tree.query(fat_aabb, |shape_index_b| {
                if shape_index_a != shape_index_b {
                    callback(shape_index_a, shape_index_b);
                }
                true
            });

            // Query static tree
            self.static_tree.query(fat_aabb, |shape_index_b| {
                callback(shape_index_a, shape_index_b);
                true
            });
        }

        self.move_list.clear();
    }
}
