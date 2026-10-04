// SPDX-FileCopyrightText: 2026 Box3D Authors
// SPDX-License-Identifier: MIT

use box3d::collision::dynamic_tree::*;
use box3d::math::*;

#[test]
fn test_dynamic_tree_insert_destroy() {
    let mut tree = DynamicTree::new();

    let aabb1 = AABB::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
    let aabb2 = AABB::new(Vec3::new(2.0, 2.0, 2.0), Vec3::new(3.0, 3.0, 3.0));

    let p1 = tree.create_proxy(aabb1, 101);
    let p2 = tree.create_proxy(aabb2, 102);

    assert_eq!(tree.get_user_data(p1), 101);
    assert_eq!(tree.get_user_data(p2), 102);

    // Query overlapping only aabb1
    let mut hits = Vec::new();
    tree.query(AABB::new(Vec3::new(0.5, 0.5, 0.5), Vec3::new(0.6, 0.6, 0.6)), |ud| {
        hits.push(ud);
        true
    });
    assert_eq!(hits, vec![101]);

    // Query overlapping both
    let mut hits_all = Vec::new();
    tree.query(AABB::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(5.0, 5.0, 5.0)), |ud| {
        hits_all.push(ud);
        true
    });
    assert_eq!(hits_all.len(), 2);
    assert!(hits_all.contains(&101));
    assert!(hits_all.contains(&102));

    // Destroy p1
    tree.destroy_proxy(p1);

    let mut hits_after = Vec::new();
    tree.query(AABB::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(5.0, 5.0, 5.0)), |ud| {
        hits_after.push(ud);
        true
    });
    assert_eq!(hits_after, vec![102]);
}

#[test]
fn test_dynamic_tree_brute_force_query() {
    let mut tree = DynamicTree::new();
    let mut boxes = Vec::new();
    let mut proxies = Vec::new();

    // Create 100 deterministic pseudo-random boxes
    let mut seed = 123456789u32;
    let mut next_rand = || {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        (seed >> 8) as f32 / (1 << 24) as f32
    };

    for i in 0..100 {
        let cx = next_rand() * 20.0 - 10.0;
        let cy = next_rand() * 20.0 - 10.0;
        let cz = next_rand() * 20.0 - 10.0;
        let extent = 0.2 + next_rand() * 0.8;

        let aabb = AABB::new(
            Vec3::new(cx - extent, cy - extent, cz - extent),
            Vec3::new(cx + extent, cy + extent, cz + extent),
        );
        boxes.push(aabb);
        let proxy = tree.create_proxy(aabb, i);
        proxies.push(proxy);
    }

    // Run 20 random query boxes and compare tree results with brute force
    for _ in 0..20 {
        let qx = next_rand() * 20.0 - 10.0;
        let qy = next_rand() * 20.0 - 10.0;
        let qz = next_rand() * 20.0 - 10.0;
        let query_box = AABB::new(
            Vec3::new(qx - 2.0, qy - 2.0, qz - 2.0),
            Vec3::new(qx + 2.0, qy + 2.0, qz + 2.0),
        );

        let mut tree_hits = Vec::new();
        tree.query(query_box, |ud| {
            tree_hits.push(ud);
            true
        });
        tree_hits.sort_unstable();

        let mut brute_hits = Vec::new();
        for (i, &proxy) in proxies.iter().enumerate() {
            let fat_aabb = tree.get_fat_aabb(proxy);
            if query_box.overlaps(fat_aabb) {
                brute_hits.push(i as i32);
            }
        }
        brute_hits.sort_unstable();

        assert_eq!(tree_hits, brute_hits);
    }
}
