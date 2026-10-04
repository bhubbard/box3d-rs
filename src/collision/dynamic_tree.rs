// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Dynamic AABB Tree for efficient 3D broadphase collision and spatial queries.
//!
//! Employs the Surface Area Heuristic (SAH) for insertion and tree rebalancing rotations.

use crate::math::{AABB, Vec3};

pub const NULL_NODE: i32 = -1;

#[derive(Clone, Debug)]
pub struct TreeNode {
    pub aabb: AABB,
    pub user_data: i32,
    pub parent: i32,
    pub child1: i32,
    pub child2: i32,
    pub height: i32,
}

impl Default for TreeNode {
    fn default() -> Self {
        Self {
            aabb: AABB::default(),
            user_data: -1,
            parent: NULL_NODE,
            child1: NULL_NODE,
            child2: NULL_NODE,
            height: -1,
        }
    }
}

impl TreeNode {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.child1 == NULL_NODE
    }
}

/// Dynamic AABB Tree structure for broadphase queries.
#[derive(Clone, Debug, Default)]
pub struct DynamicTree {
    root: i32,
    nodes: Vec<TreeNode>,
    free_list: i32,
    node_count: usize,
}

impl DynamicTree {
    pub fn new() -> Self {
        Self {
            root: NULL_NODE,
            nodes: Vec::new(),
            free_list: NULL_NODE,
            node_count: 0,
        }
    }

    fn allocate_node(&mut self) -> i32 {
        if self.free_list != NULL_NODE {
            let node_id = self.free_list;
            self.free_list = self.nodes[node_id as usize].parent;
            self.nodes[node_id as usize] = TreeNode::default();
            self.node_count += 1;
            node_id
        } else {
            let node_id = self.nodes.len() as i32;
            self.nodes.push(TreeNode::default());
            self.node_count += 1;
            node_id
        }
    }

    fn free_node(&mut self, node_id: i32) {
        let idx = node_id as usize;
        self.nodes[idx].parent = self.free_list;
        self.nodes[idx].height = -1;
        self.free_list = node_id;
        self.node_count -= 1;
    }

    /// Create a new proxy with a fattened AABB and user data
    pub fn create_proxy(&mut self, aabb: AABB, user_data: i32) -> i32 {
        let proxy_id = self.allocate_node();
        // Fatten AABB slightly
        let margin = Vec3::new(0.02, 0.02, 0.02);
        self.nodes[proxy_id as usize].aabb = AABB::new(aabb.lower_bound - margin, aabb.upper_bound + margin);
        self.nodes[proxy_id as usize].user_data = user_data;
        self.nodes[proxy_id as usize].height = 0;

        self.insert_leaf(proxy_id);
        proxy_id
    }

    /// Destroy a proxy
    pub fn destroy_proxy(&mut self, proxy_id: i32) {
        self.remove_leaf(proxy_id);
        self.free_node(proxy_id);
    }

    /// Move a proxy to a new AABB
    pub fn move_proxy(&mut self, proxy_id: i32, aabb: AABB, displacement: Vec3) -> bool {
        let node_aabb = &self.nodes[proxy_id as usize].aabb;
        if node_aabb.contains(aabb) {
            return false;
        }

        self.remove_leaf(proxy_id);

        let margin = Vec3::new(0.02, 0.02, 0.02);
        let mut fat_aabb = AABB::new(aabb.lower_bound - margin, aabb.upper_bound + margin);
        // Predict movement
        if displacement.x < 0.0 {
            fat_aabb.lower_bound.x += displacement.x * 2.0;
        } else {
            fat_aabb.upper_bound.x += displacement.x * 2.0;
        }
        if displacement.y < 0.0 {
            fat_aabb.lower_bound.y += displacement.y * 2.0;
        } else {
            fat_aabb.upper_bound.y += displacement.y * 2.0;
        }
        if displacement.z < 0.0 {
            fat_aabb.lower_bound.z += displacement.z * 2.0;
        } else {
            fat_aabb.upper_bound.z += displacement.z * 2.0;
        }

        self.nodes[proxy_id as usize].aabb = fat_aabb;
        self.insert_leaf(proxy_id);
        true
    }

    pub fn get_user_data(&self, proxy_id: i32) -> i32 {
        self.nodes[proxy_id as usize].user_data
    }

    pub fn get_fat_aabb(&self, proxy_id: i32) -> AABB {
        self.nodes[proxy_id as usize].aabb
    }

    fn insert_leaf(&mut self, leaf: i32) {
        if self.root == NULL_NODE {
            self.root = leaf;
            self.nodes[leaf as usize].parent = NULL_NODE;
            return;
        }

        // Find the best sibling using Surface Area Heuristic
        let leaf_aabb = self.nodes[leaf as usize].aabb;
        let mut index = self.root;

        while !self.nodes[index as usize].is_leaf() {
            let child1 = self.nodes[index as usize].child1;
            let child2 = self.nodes[index as usize].child2;

            let area = self.nodes[index as usize].aabb.surface_area();
            let combined_aabb = self.nodes[index as usize].aabb.union(leaf_aabb);
            let combined_area = combined_aabb.surface_area();

            // Cost of creating a new parent for this node and the new leaf
            let cost = 2.0 * combined_area;

            // Minimum cost of pushing the leaf further down the tree
            let inheritance_cost = 2.0 * (combined_area - area);

            // Cost of descending into child1
            let cost1 = if self.nodes[child1 as usize].is_leaf() {
                leaf_aabb.union(self.nodes[child1 as usize].aabb).surface_area() + inheritance_cost
            } else {
                let old_area = self.nodes[child1 as usize].aabb.surface_area();
                let new_area = leaf_aabb.union(self.nodes[child1 as usize].aabb).surface_area();
                (new_area - old_area) + inheritance_cost
            };

            // Cost of descending into child2
            let cost2 = if self.nodes[child2 as usize].is_leaf() {
                leaf_aabb.union(self.nodes[child2 as usize].aabb).surface_area() + inheritance_cost
            } else {
                let old_area = self.nodes[child2 as usize].aabb.surface_area();
                let new_area = leaf_aabb.union(self.nodes[child2 as usize].aabb).surface_area();
                (new_area - old_area) + inheritance_cost
            };

            // Descend according to the minimum cost
            if cost < cost1 && cost < cost2 {
                break;
            }

            index = if cost1 < cost2 { child1 } else { child2 };
        }

        let sibling = index;

        // Create a new parent
        let old_parent = self.nodes[sibling as usize].parent;
        let new_parent = self.allocate_node();
        self.nodes[new_parent as usize].parent = old_parent;
        self.nodes[new_parent as usize].user_data = -1;
        self.nodes[new_parent as usize].aabb = leaf_aabb.union(self.nodes[sibling as usize].aabb);
        self.nodes[new_parent as usize].height = self.nodes[sibling as usize].height + 1;

        if old_parent != NULL_NODE {
            // The sibling was not the root
            if self.nodes[old_parent as usize].child1 == sibling {
                self.nodes[old_parent as usize].child1 = new_parent;
            } else {
                self.nodes[old_parent as usize].child2 = new_parent;
            }
            self.nodes[new_parent as usize].child1 = sibling;
            self.nodes[new_parent as usize].child2 = leaf;
            self.nodes[sibling as usize].parent = new_parent;
            self.nodes[leaf as usize].parent = new_parent;
        } else {
            // The sibling was the root
            self.nodes[new_parent as usize].child1 = sibling;
            self.nodes[new_parent as usize].child2 = leaf;
            self.nodes[sibling as usize].parent = new_parent;
            self.nodes[leaf as usize].parent = new_parent;
            self.root = new_parent;
        }

        // Walk back up the tree refitting AABBs and heights
        let mut index = self.nodes[leaf as usize].parent;
        while index != NULL_NODE {
            index = self.balance(index);

            let child1 = self.nodes[index as usize].child1;
            let child2 = self.nodes[index as usize].child2;

            self.nodes[index as usize].height =
                1 + self.nodes[child1 as usize].height.max(self.nodes[child2 as usize].height);
            self.nodes[index as usize].aabb =
                self.nodes[child1 as usize].aabb.union(self.nodes[child2 as usize].aabb);

            index = self.nodes[index as usize].parent;
        }
    }

    fn remove_leaf(&mut self, leaf: i32) {
        if leaf == self.root {
            self.root = NULL_NODE;
            return;
        }

        let parent = self.nodes[leaf as usize].parent;
        let grand_parent = self.nodes[parent as usize].parent;
        let sibling = if self.nodes[parent as usize].child1 == leaf {
            self.nodes[parent as usize].child2
        } else {
            self.nodes[parent as usize].child1
        };

        if grand_parent != NULL_NODE {
            // Destroy parent and connect sibling to grandParent
            if self.nodes[grand_parent as usize].child1 == parent {
                self.nodes[grand_parent as usize].child1 = sibling;
            } else {
                self.nodes[grand_parent as usize].child2 = sibling;
            }
            self.nodes[sibling as usize].parent = grand_parent;
            self.free_node(parent);

            // Adjust ancestor bounds
            let mut index = grand_parent;
            while index != NULL_NODE {
                index = self.balance(index);

                let child1 = self.nodes[index as usize].child1;
                let child2 = self.nodes[index as usize].child2;

                self.nodes[index as usize].aabb =
                    self.nodes[child1 as usize].aabb.union(self.nodes[child2 as usize].aabb);
                self.nodes[index as usize].height =
                    1 + self.nodes[child1 as usize].height.max(self.nodes[child2 as usize].height);

                index = self.nodes[index as usize].parent;
            }
        } else {
            self.root = sibling;
            self.nodes[sibling as usize].parent = NULL_NODE;
            self.free_node(parent);
        }
    }

    fn balance(&mut self, i_a: i32) -> i32 {
        if self.nodes[i_a as usize].is_leaf() || self.nodes[i_a as usize].height < 2 {
            return i_a;
        }

        let i_b = self.nodes[i_a as usize].child1;
        let i_c = self.nodes[i_a as usize].child2;

        let b_height = self.nodes[i_b as usize].height;
        let c_height = self.nodes[i_c as usize].height;
        let balance = c_height - b_height;

        // Rotate C up
        if balance > 1 {
            let i_f = self.nodes[i_c as usize].child1;
            let i_g = self.nodes[i_c as usize].child2;
            let f_height = self.nodes[i_f as usize].height;
            let g_height = self.nodes[i_g as usize].height;

            self.nodes[i_c as usize].child1 = i_a;
            self.nodes[i_c as usize].parent = self.nodes[i_a as usize].parent;
            self.nodes[i_a as usize].parent = i_c;

            let c_parent = self.nodes[i_c as usize].parent;
            if c_parent != NULL_NODE {
                let p = c_parent as usize;
                if self.nodes[p].child1 == i_a {
                    self.nodes[p].child1 = i_c;
                } else {
                    self.nodes[p].child2 = i_c;
                }
            } else {
                self.root = i_c;
            }

            // Rotate
            if f_height > g_height {
                self.nodes[i_c as usize].child2 = i_f;
                self.nodes[i_a as usize].child2 = i_g;
                self.nodes[i_g as usize].parent = i_a;
                self.nodes[i_a as usize].aabb =
                    self.nodes[i_b as usize].aabb.union(self.nodes[i_g as usize].aabb);
                self.nodes[i_c as usize].aabb =
                    self.nodes[i_a as usize].aabb.union(self.nodes[i_f as usize].aabb);

                self.nodes[i_a as usize].height =
                    1 + self.nodes[i_b as usize].height.max(self.nodes[i_g as usize].height);
                self.nodes[i_c as usize].height =
                    1 + self.nodes[i_a as usize].height.max(self.nodes[i_f as usize].height);
            } else {
                self.nodes[i_c as usize].child2 = i_g;
                self.nodes[i_a as usize].child2 = i_f;
                self.nodes[i_f as usize].parent = i_a;
                self.nodes[i_a as usize].aabb =
                    self.nodes[i_b as usize].aabb.union(self.nodes[i_f as usize].aabb);
                self.nodes[i_c as usize].aabb =
                    self.nodes[i_a as usize].aabb.union(self.nodes[i_g as usize].aabb);

                self.nodes[i_a as usize].height =
                    1 + self.nodes[i_b as usize].height.max(self.nodes[i_f as usize].height);
                self.nodes[i_c as usize].height =
                    1 + self.nodes[i_a as usize].height.max(self.nodes[i_g as usize].height);
            }

            return i_c;
        }

        // Rotate B up
        if balance < -1 {
            let i_d = self.nodes[i_b as usize].child1;
            let i_e = self.nodes[i_b as usize].child2;
            let d_height = self.nodes[i_d as usize].height;
            let e_height = self.nodes[i_e as usize].height;

            self.nodes[i_b as usize].child1 = i_a;
            self.nodes[i_b as usize].parent = self.nodes[i_a as usize].parent;
            self.nodes[i_a as usize].parent = i_b;

            let b_parent = self.nodes[i_b as usize].parent;
            if b_parent != NULL_NODE {
                let p = b_parent as usize;
                if self.nodes[p].child1 == i_a {
                    self.nodes[p].child1 = i_b;
                } else {
                    self.nodes[p].child2 = i_b;
                }
            } else {
                self.root = i_b;
            }

            if d_height > e_height {
                self.nodes[i_b as usize].child2 = i_d;
                self.nodes[i_a as usize].child1 = i_e;
                self.nodes[i_e as usize].parent = i_a;
                self.nodes[i_a as usize].aabb =
                    self.nodes[i_c as usize].aabb.union(self.nodes[i_e as usize].aabb);
                self.nodes[i_b as usize].aabb =
                    self.nodes[i_a as usize].aabb.union(self.nodes[i_d as usize].aabb);

                self.nodes[i_a as usize].height =
                    1 + self.nodes[i_c as usize].height.max(self.nodes[i_e as usize].height);
                self.nodes[i_b as usize].height =
                    1 + self.nodes[i_a as usize].height.max(self.nodes[i_d as usize].height);
            } else {
                self.nodes[i_b as usize].child2 = i_e;
                self.nodes[i_a as usize].child1 = i_d;
                self.nodes[i_d as usize].parent = i_a;
                self.nodes[i_a as usize].aabb =
                    self.nodes[i_c as usize].aabb.union(self.nodes[i_d as usize].aabb);
                self.nodes[i_b as usize].aabb =
                    self.nodes[i_a as usize].aabb.union(self.nodes[i_e as usize].aabb);

                self.nodes[i_a as usize].height =
                    1 + self.nodes[i_c as usize].height.max(self.nodes[i_d as usize].height);
                self.nodes[i_b as usize].height =
                    1 + self.nodes[i_a as usize].height.max(self.nodes[i_e as usize].height);
            }

            return i_b;
        }

        i_a
    }

    /// Query tree for all proxies overlapping given AABB
    pub fn query<F>(&self, aabb: AABB, mut callback: F)
    where
        F: FnMut(i32) -> bool,
    {
        if self.root == NULL_NODE {
            return;
        }

        let mut stack = Vec::with_capacity(64);
        stack.push(self.root);

        while let Some(node_id) = stack.pop() {
            let node = &self.nodes[node_id as usize];
            if node.aabb.overlaps(aabb) {
                if node.is_leaf() {
                    let proceed = callback(node.user_data);
                    if !proceed {
                        return;
                    }
                } else {
                    stack.push(node.child1);
                    stack.push(node.child2);
                }
            }
        }
    }
}
