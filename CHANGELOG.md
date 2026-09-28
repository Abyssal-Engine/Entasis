# Changelog

## 1.0.0

### Physics

- Single-precision 3D rigid-body simulation with dynamic and kinematic bodies, static colliders, sleeping, multithreaded stepping and continuous collision detection
- Primitive shapes, convex hulls, nested compounds and triangle meshes, with offline cooking and world import
- Joints, motors and servos, physical axis locks, force/torque reaction readback and optional automatic joint breaking
- Collision filtering, friction, restitution, contact events, trigger events and per-instance mixed solid/sensor parts
- Ray, sweep, overlap, closest-point, distance and separation queries, with batching and independently owned query contexts
- Queued forces and torques, per-body damping and kinematic targets

### Integration

- Native Odin API and C ABI with C11/C++20 consumers, separate runtime and cooking libraries, and shared and static linking
- Custom shape, collision, sweep and constraint registration, application-owned callbacks and external worker dispatchers
- Capacity reservation, allocator selection, deferred structural commands, state views and profiling
- Task-organized headless examples and API references covering ownership, threading, lifetimes and failure behavior

### Tools

- Optional raylib/raygui physics viewer with live examples, controller input, camera controls and step-by-step diagnostics
- Saved benchmark results with recording status, indexed replay, seeking and side-by-side comparison
- Explicit benchmark Build and Run commands, with viewer launch progress, cancellation and retained logs
- LZ4-compressed recordings shared by native Windows and Linux tools. The viewer is available from the full checkout, not the C SDK or Odin source packages

### Platforms and distribution

- Windows and Linux AMD64 builds targeting the full `x86-64-v3` CPU baseline, including AVX2
- Native SDKs with headers, libraries, CMake integration, runnable C/C++ examples and Linux pkg-config metadata
- Source ZIP and TAR.GZ packages with build commands, examples, documentation and licenses

See [platform support and limitations](docs/LIMITS.md) for qualified systems and behavior boundaries
