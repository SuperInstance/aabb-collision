# AABB Collision Detection

**Axis-Aligned Bounding Box (AABB)** collision detection is the simplest and fastest method for detecting overlaps between rectangular shapes whose axes are aligned with the coordinate system.

## Why It Matters

AABB is the backbone of broad-phase collision detection in game engines, physics simulations, and spatial partitioning. It's O(1) per pair check — just compare min/max bounds on each axis. When you have thousands of objects, AABB narrow-phase filtering eliminates most non-colliding pairs before running expensive polygon-level checks.

## How It Works

Two AABBs overlap if and only if they overlap on every axis. For each dimension: `a.min <= b.max AND a.max >= b.min`. This implementation uses the separating axis theorem for axis-aligned boxes, which reduces to 4 comparisons in 2D or 6 in 3D.

## Usage

```toml
[dependencies]
aabb-collision = "0.1.0"
```

```rust
use aabb_collision;

// See examples/ directory for detailed usage
```

## API

- `AABB` (lib.rs)
- `CollisionDetector` (lib.rs)
- `AABB3` (lib.rs)

## Architecture

This crate is part of the **[SuperInstance](https://github.com/SuperInstance)** ecosystem — a conservation-law-based framework for fleet coordination, ternary computation, and distributed agent systems.

### Related Crates

- [`superinstance-core`](https://github.com/SuperInstance/superinstance-core) — Core conservation law (γ + η = C)
- [`superinstance-harness`](https://github.com/SuperInstance/superinstance-harness) — Build harness and self-improving loop
- [`fleet-coordinator`](https://github.com/SuperInstance/fleet-coordinator) — Fleet-level coordination

## References

- [SuperInstance Architecture](https://github.com/SuperInstance/SuperInstance/blob/main/ARCHITECTURE.md)
- [Conservation Law Paper](https://github.com/SuperInstance/SuperInstance/blob/main/docs/conservation-law.md)

## License

MIT
