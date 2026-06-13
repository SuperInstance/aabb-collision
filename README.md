# AABB Collision Detection

**Axis-Aligned Bounding Box (AABB) Collision Detection** is a Rust library providing efficient 2D and 3D intersection testing, overlap computation, and broad-phase collision detection using sweep-and-prune on axis-aligned bounding boxes.

## Why It Matters

Collision detection is the computational bottleneck in physics engines, game loops, and robotics simulation. For N objects, naive pairwise testing requires O(N²) comparisons — prohibitive at scale. AABB-based broad-phase culling reduces this to O(N·log N) or better by quickly eliminating pairs that cannot possibly intersect. The AABB representation is the most common spatial pruning structure because intersection testing reduces to 2–3 coordinate comparisons per axis, each an O(1) operation. Real-time simulation engines (Unreal, Unity, Box2D) all use AABB trees as their first-pass collision filter. This crate provides a clean, zero-dependency implementation suitable for both game development and scientific simulation.

## How It Works

An AABB is defined by its minimum and maximum corner coordinates. Two AABBs `A` and `B` intersect if and only if they overlap on every axis:

```
A.min.x ≤ B.max.x  AND  A.max.x ≥ B.min.x
A.min.y ≤ B.max.y  AND  A.max.y ≥ B.min.y
```

For the 3D case, a third z-axis test is added. Each test is O(1) — six floating-point comparisons total for 2D, nine for 3D.

**Sweep and Prune (Sort and Sweep):**
The broad-phase `CollisionDetector` projects all AABBs onto a primary axis and sorts them. Two AABBs can only overlap if their projections overlap, so the algorithm:

1. Sorts objects by their minimum x-coordinate: O(N log N)
2. Sweeps from left to right, maintaining an active set
3. For each new object, tests against only the active set: O(N + k) where k = collision pairs

Worst case remains O(N²) when all objects overlap on the sweep axis, but typical scenes achieve near-linear performance.

**Key operations and complexity:**

| Operation | Time | Notes |
|-----------|------|-------|
| `intersects` | O(1) | 2–3 comparisons per axis |
| `contains` | O(1) | 4 comparisons |
| `overlap` | O(1) | Per-axis max/min |
| `find_collisions` | O(N²) worst | Naive pairwise in this impl |
| Sweep-and-prune | O(N log N + k) | Typical broad-phase |

The overlap region is computed as the component-wise maximum of the minima and minimum of the maxima — if the resulting box has non-negative extent on all axes, it is the valid intersection AABB.

## Quick Start

```rust
use aabb_collision::{AABB, CollisionDetector};

fn main() {
    let a = AABB::new([0.0, 0.0], [2.0, 2.0]);
    let b = AABB::from_center([1.5, 1.5], [1.0, 1.0]);

    assert!(a.intersects(&b));
    let overlap = a.overlap(&b).unwrap();
    println!("Overlap region: {:?} to {:?}", overlap.min, overlap.max);

    let mut detector = CollisionDetector::new();
    detector.insert(0, AABB::new([0.0, 0.0], [2.0, 2.0]));
    detector.insert(1, AABB::new([1.0, 1.0], [3.0, 3.0]));
    detector.insert(2, AABB::new([10.0, 10.0], [11.0, 11.0]));

    let pairs = detector.find_collisions();
    println!("Collision pairs: {:?}", pairs); // [(0, 1)]
}
```

## API

| Type/Function | Description |
|---------------|-------------|
| `AABB` | 2D axis-aligned bounding box with min/max corners |
| `AABB3` | 3D variant with z-axis support |
| `AABB::from_center` | Construct from center point and half-extents |
| `AABB::intersects` | Boolean overlap test, O(1) |
| `AABB::contains` | Strict containment test |
| `AABB::overlap` | Returns intersection AABB or None |
| `AABB::merged` | Union (minimum enclosing) of two AABBs |
| `CollisionDetector` | Broad-phase manager with insert/find_collisions |
| `AABB::area` / `AABB3::volume` | Geometric measure |

## Architecture Notes

AABB collision serves as the **spatial pruning layer** in the SuperInstance fleet's physics-aware agent simulation. When evaluating γ + η = C conservation, the spatial distribution of agents (their bounding boxes) determines interaction graphs for collision avoidance behaviors. Fast broad-phase culling ensures the simulation scales to thousands of agents without degrading the conservation-law verification loop.

See [ARCHITECTURE.md](https://github.com/SuperInstance/SuperInstance/blob/main/ARCHITECTURE.md) for fleet topology.

## References

1. Ericson, C. (2004). *Real-Time Collision Detection*. Morgan Kaufmann. Chapter 2: Bounding Volumes.
2. van den Bergen, G. (1997). "Efficient Collision Detection of Complex Deformable Models using AABB Trees." *Journal of Graphics Tools*, 2(4), 1–14.

## License

MIT
