// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

//! Generational identifiers for physics entities (World, Body, Shape, Joint).
//!
//! Using generational indices prevents use-after-free bugs and dangling references
//! when entities are destroyed and reused.

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WorldId {
    pub index1: u16,
    pub generation: u16,
}

impl WorldId {
    pub const NULL: Self = Self {
        index1: 0,
        generation: 0,
    };

    #[inline]
    pub fn is_valid(self) -> bool {
        self.index1 != 0
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BodyId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl BodyId {
    pub const NULL: Self = Self {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    #[inline]
    pub fn is_valid(self) -> bool {
        self.index1 > 0
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ShapeId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl ShapeId {
    pub const NULL: Self = Self {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    #[inline]
    pub fn is_valid(self) -> bool {
        self.index1 > 0
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JointId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl JointId {
    pub const NULL: Self = Self {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    #[inline]
    pub fn is_valid(self) -> bool {
        self.index1 > 0
    }
}

/// Generational Id pool for slot reuse
#[derive(Debug, Clone)]
pub struct IdPool {
    free_list: Vec<usize>,
    generations: Vec<u16>,
}

impl Default for IdPool {
    fn default() -> Self {
        Self::new()
    }
}

impl IdPool {
    pub fn new() -> Self {
        Self {
            free_list: Vec::new(),
            generations: Vec::new(),
        }
    }

    pub fn allocate(&mut self) -> (usize, u16) {
        if let Some(index) = self.free_list.pop() {
            let gen = self.generations[index];
            (index, gen)
        } else {
            let index = self.generations.len();
            let gen = 1;
            self.generations.push(gen);
            (index, gen)
        }
    }

    pub fn free(&mut self, index: usize) {
        if index < self.generations.len() {
            self.generations[index] = self.generations[index].wrapping_add(1);
            if self.generations[index] == 0 {
                self.generations[index] = 1;
            }
            self.free_list.push(index);
        }
    }

    pub fn is_valid(&self, index: usize, generation: u16) -> bool {
        index < self.generations.len() && self.generations[index] == generation
    }
}
