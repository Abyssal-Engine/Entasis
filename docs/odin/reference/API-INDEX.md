# Odin API index

Find declarations by domain below. Each symbol links to its usage guide, and each source-file heading links to the current declarations

## Domains

- [World](#world)
- [Bodies and statics](#bodies-and-statics)
- [Structural operations](#structural-operations)
- [Shapes](#shapes)
- [Constraints](#constraints)
- [Collision](#collision)
- [Queries](#queries)
- [Data access](#data-access)
- [Callbacks and stages](#callbacks-and-stages)
- [Profiling and inspection](#profiling-and-inspection)

## World

Reference: [WORLD.md](WORLD.md)

### [world.odin](../../../src/entasis/world.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`World`](WORLD.md) | distinct type | Caller-owned opaque handle for exactly one low-level simulation |
| [`world_init`](WORLD.md) | procedure | Creates one world with a facade-owned Buffer_Pool |
| [`world_init_with_pool`](WORLD.md) | procedure | Creates one world using a caller-owned Buffer_Pool |
| [`world_step`](WORLD.md) | procedure | Advance one timestep and wait for physics workers |
| [`world_clear`](WORLD.md) | procedure | Remove simulation contents while retaining allocated capacity |
| [`world_ensure_capacity`](WORLD.md) | procedure | Grows world storage to satisfy the supplied initial capacity hints |
| [`world_resize`](WORLD.md) | procedure | Grows or shrinks retained world storage toward the supplied capacity hints |
| [`world_destroy`](WORLD.md) | procedure | Releases the simulation and every facade-owned resource |

### [world_description.odin](../../../src/entasis/world_description.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Substep_Velocity_Iteration_Scheduler_Proc`](WORLD.md) | type alias | Selects a positive velocity iteration count for one substep |
| [`Damping`](WORLD.md) | struct | Contains the per-second damping fractions used by the default pose integrator |
| [`Capacity_Hints`](WORLD.md) | struct | Contains initial storage hints |
| [`Solve_Description`](WORLD.md) | struct | Exposes the stable built-in solve settings |
| [`Threading_Description`](WORLD.md) | struct | Select caller-only, included-worker or external-dispatcher execution |
| [`World_Description`](WORLD.md) | struct | World initialization settings, callbacks and allocation policy |
| [`capacity_hints_default`](WORLD.md) | procedure | Returns the low-level default initial capacities while omitting settings derived from solve and threading policy |
| [`solve_description_default`](WORLD.md) | procedure | Returns the existing solver defaults |
| [`threading_description_default`](WORLD.md) | procedure | Selects a caller-thread-only world and creates no background worker |
| [`world_description_default`](WORLD.md) | procedure | Returns a complete ordinary world description |

### [timestep.odin](../../../src/entasis/timestep.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Fixed_Stepper`](WORLD.md) | struct | Persistent frame-time accumulator with a catch-up step limit |
| [`fixed_stepper`](WORLD.md) | procedure | Constructs caller-owned fixed-step state |
| [`fixed_stepper_reset`](WORLD.md) | procedure | Clears accumulated elapsed time without modifying policy |
| [`fixed_stepper_update`](WORLD.md) | procedure | Accumulate elapsed time and run fixed-duration steps |
| [`solve_description_substeps`](WORLD.md) | procedure | Constructs a fixed-iteration substep policy |
| [`solve_description_substep_scheduler`](WORLD.md) | procedure | Constructs a substep policy whose iteration count is selected by a caller-owned allocation-free callback |

### [dispatcher.odin](../../../src/entasis/dispatcher.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Dispatcher_Status`](WORLD.md) | enum | Describes external worker-pool failures without conflating scheduler state with physics operation status |
| [`Thread_Pool`](WORLD.md) | type alias | Included caller-owned worker pool implementation |
| [`Dispatcher_Interface`](WORLD.md) | struct | Application-supplied blocking worker dispatcher |
| [`Dispatcher_Work_Proc`](WORLD.md) | procedure | Worker callback, invoked once per worker index during a blocking dispatch |
| [`Dispatcher_Dispatch_Proc`](WORLD.md) | procedure | Run every worker callback and wait for all of them before returning |
| [`Dispatcher_Worker_Pool_Proc`](WORLD.md) | procedure | Return the requested worker's exclusive buffer pool |
| [`dispatcher_interface_validate`](WORLD.md) | procedure | Verifies the stable blocking-dispatch contract |
| [`thread_pool_init`](WORLD.md) | procedure | Initializes the included caller-owned worker pool |
| [`thread_pool_destroy`](WORLD.md) | procedure | Stops and releases an included caller-owned worker pool |
| [`dispatcher_from_thread_pool`](WORLD.md) | procedure | Exposes an included caller-owned Thread_Pool through the stable external interface |
| [`world_step_external`](WORLD.md) | procedure | Advances one timestep through a caller-owned blocking dispatcher interface |

## Bodies and statics

Reference: [BODIES-STATICS.md](BODIES-STATICS.md)

### [bodies.odin](../../../src/entasis/bodies.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`pose`](BODIES-STATICS.md) | procedure | Constructs a rigid pose |
| [`velocity`](BODIES-STATICS.md) | procedure | Constructs body linear and angular velocity |
| [`body_activity`](BODIES-STATICS.md) | procedure | Constructs sleeping thresholds |
| [`body_activity_default`](BODIES-STATICS.md) | procedure | Returns the ordinary facade sleep policy |
| [`collidable`](BODIES-STATICS.md) | procedure | Constructs a body collidable description |
| [`body_dynamic`](BODIES-STATICS.md) | procedure | Constructs a dynamic collidable body description |
| [`body_kinematic`](BODIES-STATICS.md) | procedure | Constructs a kinematic collidable body description |
| [`body_shapeless`](BODIES-STATICS.md) | procedure | Constructs a dynamic body with no broad-phase collidable |
| [`body_add`](BODIES-STATICS.md) | procedure | Adds one body |
| [`body_remove`](BODIES-STATICS.md) | procedure | Removes a body and every connected constraint |
| [`body_get`](BODIES-STATICS.md) | procedure | Copy an active or sleeping body's state |
| [`body_apply`](BODIES-STATICS.md) | procedure | Replaces the complete body description |
| [`body_set_pose`](BODIES-STATICS.md) | procedure | Updates the pose through the complete body mutation path |
| [`body_set_velocity`](BODIES-STATICS.md) | procedure | Updates velocity and awakens a sleeping body |
| [`body_set_inertia`](BODIES-STATICS.md) | procedure | Updates local inertia and handles dynamic/kinematic changes |
| [`body_set_activity`](BODIES-STATICS.md) | procedure | Updates sleep thresholds and resets the activity counters |
| [`body_set_collidable`](BODIES-STATICS.md) | procedure | Updates shape, continuity, and speculative margins |
| [`body_set_shape`](BODIES-STATICS.md) | procedure | Updates only the shape while retaining the other collidable settings |

### [body_interaction.odin](../../../src/entasis/body_interaction.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`body_apply_linear_impulse`](BODIES-STATICS.md) | procedure | Applies an instantaneous linear impulse without awakening the body |
| [`body_apply_angular_impulse`](BODIES-STATICS.md) | procedure | Applies an instantaneous angular impulse using the body's current world inverse inertia tensor without awakening the body |
| [`body_apply_impulse`](BODIES-STATICS.md) | procedure | Applies an impulse at a world-space offset from the body center of mass |
| [`body_velocity_at_offset`](BODIES-STATICS.md) | procedure | Returns the world-space velocity at an offset from the body's center of mass |
| [`body_bounds`](BODIES-STATICS.md) | procedure | Returns the current broad-phase bounds of a collidable body |
| [`body_update_bounds`](BODIES-STATICS.md) | procedure | Recomputes one body's broad-phase bounds after an advanced direct pose write |
| [`static_bounds`](BODIES-STATICS.md) | procedure | Returns one static's current broad-phase bounds |
| [`static_update_bounds`](BODIES-STATICS.md) | procedure | Recomputes one static's broad-phase bounds after an advanced direct pose write |

### [statics.odin](../../../src/entasis/statics.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Awakening_Policy`](BODIES-STATICS.md) | enum | Selects whether a static mutation wakes sleeping dynamic bodies whose bounds overlap the old or new static bounds |
| [`Static_Awakening_Filter_Proc`](BODIES-STATICS.md) | type alias | Selects sleeping dynamic bodies whose islands may be awakened by a static mutation |
| [`static_body`](BODIES-STATICS.md) | procedure | Constructs a static description with discrete continuity |
| [`static_add`](BODIES-STATICS.md) | procedure | Adds one static and applies the selected awakening policy |
| [`static_remove`](BODIES-STATICS.md) | procedure | Removes one static and applies the selected awakening policy |
| [`static_get`](BODIES-STATICS.md) | procedure | Copy one static's description |
| [`static_apply`](BODIES-STATICS.md) | procedure | Replaces the complete static description and applies the selected awakening policy to overlapping sleeping bodies |
| [`static_set_pose`](BODIES-STATICS.md) | procedure | Updates only the pose |
| [`static_set_shape`](BODIES-STATICS.md) | procedure | Updates only the shape |
| [`static_set_continuity`](BODIES-STATICS.md) | procedure | Updates only continuous-detection settings |
| [`static_add_filtered`](BODIES-STATICS.md) | procedure | Adds a static and awakens only overlapping sleeping islands containing at least one body accepted by filter |
| [`static_remove_filtered`](BODIES-STATICS.md) | procedure | Removes a static and applies a caller-owned awakening filter to sleeping bodies overlapping the removed bounds |
| [`static_apply_filtered`](BODIES-STATICS.md) | procedure | Replaces a static and filters awakening against the union of its old and new bounds |

### [sleeping.odin](../../../src/entasis/sleeping.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Body_Activation_State`](BODIES-STATICS.md) | enum | Reports whether a live body is stored in the active set or in one sleeping island |
| [`body_activation_state`](BODIES-STATICS.md) | procedure | Reports the storage state of one live body without exposing inactive-set indices or internal island storage |
| [`body_is_active`](BODIES-STATICS.md) | procedure | Reports whether one live body is in the active simulation set |
| [`body_is_sleeping`](BODIES-STATICS.md) | procedure | Reports whether one live body is stored in a sleeping island |
| [`body_awaken`](BODIES-STATICS.md) | procedure | Awakens the complete constraint-connected island containing the body |
| [`bodies_sleep_group`](BODIES-STATICS.md) | procedure | Moves one caller-selected group of active, constraint-disconnected bodies into a sleeping island |

### [ccd.odin](../../../src/entasis/ccd.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`ccd_discrete`](BODIES-STATICS.md) | procedure | Disables sweep tests and limits velocity expansion to the collidable's speculative-margin range |
| [`ccd_passive`](BODIES-STATICS.md) | procedure | Disables sweep tests but allows velocity expansion beyond the calculated speculative margin so Continuous partners can discover the pair |
| [`ccd_continuous`](BODIES-STATICS.md) | procedure | Enables swept continuous collision detection |

### Body controls

Reference: [BODIES-STATICS.md](BODIES-STATICS.md#optional-body-controls). Source: [`body_control.odin`](../../../src/entasis/body_control.odin). Types: `Body_Control_Configuration`, `Body_Control_Wake`, `Body_Input_Mode`, `Body_Damping_Mode`, `Body_Damping`

| Operations                                                                                                              | Purpose                                                        |
| ----------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| `body_control_configuration_default`, `world_enable_body_control`, `world_disable_body_control`, `body_control_reserve` | Optional copied configuration, ownership and explicit capacity |
| `body_add_force`, `body_add_torque`, `body_add_force_at_position`, `body_clear_inputs`                                  | Next-integrated-step world inputs                              |
| `body_set_damping`, `body_get_damping`                                                                                  | Explicit per-body attenuation and callback composition         |
| `body_set_kinematic_target`, `body_get_kinematic_target`, `body_clear_kinematic_target`                                 | Velocity-derived target motion and cancellation                |

`Body_Lock_Axis`, `Body_Lock_Axes`, `Body_Axis_Lock` and `body_axis_lock_default(reference)` describe selected physical rows. `body_set_axis_lock`, `body_get_axis_lock` and `body_clear_axis_lock` configure, inspect and remove the owned joint. See [physical axis locks](BODIES-STATICS.md#physical-axis-locks)

## Structural operations

Reference: [STRUCTURAL-OPERATIONS.md](STRUCTURAL-OPERATIONS.md)

### [batch.odin](../../../src/entasis/batch.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`body_add_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Adds a prefix of descriptions in input order after one capacity check. handles must contain at least len(descriptions) entries |
| [`body_apply_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Applies matching handles and descriptions in input order |
| [`body_remove_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Removes handles in input order |
| [`static_add_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Adds a prefix of descriptions in input order after one capacity check |
| [`static_apply_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Applies matching handles and descriptions in input order |
| [`static_remove_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Removes handles in input order using one awakening policy |
| [`shape_add_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Validates and registers one homogeneous batch of built-in primitive shapes |
| [`shape_add_batch_typed`](STRUCTURAL-OPERATIONS.md) | procedure | Advanced homogeneous batch path for cooked or custom registered shape types |
| [`shape_remove_batch`](STRUCTURAL-OPERATIONS.md) | procedure | Removes unreferenced shapes in input order |

### [commands.odin](../../../src/entasis/commands.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Command_Kind`](STRUCTURAL-OPERATIONS.md) | enum | Identifies one facade structural operation |
| [`Command`](STRUCTURAL-OPERATIONS.md) | struct | Caller-owned tagged structural command |
| [`Command_Buffer`](STRUCTURAL-OPERATIONS.md) | struct | Binds caller-owned commands, optional per-command statuses, and add-result arrays |
| [`command_buffer`](STRUCTURAL-OPERATIONS.md) | procedure | Constructs a non-owning command-buffer view |
| [`command_body_add`](STRUCTURAL-OPERATIONS.md) | procedure | Constructs a body-add command with caller-selected result slot |
| [`command_body_apply`](STRUCTURAL-OPERATIONS.md) | procedure | Constructs a body-apply command |
| [`command_body_remove`](STRUCTURAL-OPERATIONS.md) | procedure | Constructs a body-remove command |
| [`command_static_add`](STRUCTURAL-OPERATIONS.md) | procedure | Constructs a static-add command with result slot and awakening policy |
| [`command_static_apply`](STRUCTURAL-OPERATIONS.md) | procedure | Constructs a static-apply command with awakening policy |
| [`command_static_remove`](STRUCTURAL-OPERATIONS.md) | procedure | Constructs a static-remove command with awakening policy |
| [`world_apply_commands`](STRUCTURAL-OPERATIONS.md) | procedure | Applies caller-owned structural commands in exact input order |

## Shapes

Reference: [SHAPES.md](SHAPES.md)

### [shapes.odin](../../../src/entasis/shapes.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`SHAPE_TYPE_INVALID`](SHAPES.md) | constant | Invalid explicit shape type sentinel |
| [`SHAPE_TYPE_SPHERE`](SHAPES.md) | constant | Built-in sphere registry type ID |
| [`SHAPE_TYPE_CAPSULE`](SHAPES.md) | constant | Built-in capsule registry type ID |
| [`SHAPE_TYPE_BOX`](SHAPES.md) | constant | Built-in box registry type ID |
| [`SHAPE_TYPE_TRIANGLE`](SHAPES.md) | constant | Built-in triangle registry type ID |
| [`SHAPE_TYPE_CYLINDER`](SHAPES.md) | constant | Built-in cylinder registry type ID |
| [`SHAPE_TYPE_CONVEX_HULL`](SHAPES.md) | constant | Built-in convex hull registry type ID |
| [`SHAPE_TYPE_COMPOUND`](SHAPES.md) | constant | Built-in compound registry type ID |
| [`SHAPE_TYPE_BIG_COMPOUND`](SHAPES.md) | constant | Built-in big compound registry type ID |
| [`SHAPE_TYPE_MESH`](SHAPES.md) | constant | Built-in mesh registry type ID |
| [`Shape_Type_ID`](SHAPES.md) | distinct type | Stable numeric identity used by the registry |
| [`Sphere`](SHAPES.md) | type alias | Sphere geometry |
| [`Capsule`](SHAPES.md) | type alias | Zero-copy built-in Y-axis capsule shape |
| [`Box`](SHAPES.md) | type alias | Zero-copy built-in box shape using half extents internally |
| [`Triangle`](SHAPES.md) | type alias | Zero-copy clockwise one-sided triangle shape |
| [`Cylinder`](SHAPES.md) | type alias | Zero-copy built-in Y-axis cylinder shape |
| [`sphere`](SHAPES.md) | procedure | Constructs a sphere from its radius |
| [`box`](SHAPES.md) | procedure | Constructs a box from full width, height, and depth |
| [`box_half_extents`](SHAPES.md) | procedure | Constructs a box from explicit half extents |
| [`capsule`](SHAPES.md) | procedure | Constructs a Y-axis capsule from radius and full cylindrical length |
| [`capsule_half_length`](SHAPES.md) | procedure | Constructs a Y-axis capsule from radius and half length |
| [`cylinder`](SHAPES.md) | procedure | Constructs a Y-axis cylinder from radius and full length |
| [`cylinder_half_length`](SHAPES.md) | procedure | Constructs a Y-axis cylinder from radius and half length |
| [`triangle`](SHAPES.md) | procedure | Constructs a clockwise one-sided triangle |
| [`shape_type_id`](SHAPES.md) | procedure | Maps supported built-in shape types to compile-time constants |
| [`shape_validate`](SHAPES.md) | procedure | Validates public built-in primitive descriptions without allocation |
| [`shape_add`](SHAPES.md) | procedure | Validates and registers one built-in primitive |
| [`shape_add_typed`](SHAPES.md) | procedure | Explicit-ID path for cooked container shapes and registered custom shapes |
| [`shape_remove`](SHAPES.md) | procedure | Removes an unreferenced shape |

### [shape_inertia.odin](../../../src/entasis/shape_inertia.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`shape_inertia`](SHAPES.md) | procedure | Computes inertia directly from a built-in primitive value |
| [`shape_registered_inertia`](SHAPES.md) | procedure | Computes inertia through the world's shape registry |

### [shape_tools.odin](../../../src/entasis/shape_tools.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Convex_Hull`](SHAPES.md) | type alias | Exact registered convex-hull storage type |
| [`Compound`](SHAPES.md) | type alias | Exact small compound storage type |
| [`Big_Compound`](SHAPES.md) | type alias | Tree-accelerated compound storage type |
| [`Mesh`](SHAPES.md) | type alias | Exact registered mesh storage type |
| [`Shape_Info`](SHAPES.md) | struct | Describes one currently registered shape without exposing mutable registry metadata |
| [`shape_inspect`](SHAPES.md) | procedure | Returns the registered type, value layout and live reference count of a shape |
| [`shape_borrow_raw`](SHAPES.md) | procedure | Returns a read-only pointer to the registered shape value |
| [`shape_borrow_typed`](SHAPES.md) | procedure | Returns a read-only typed pointer after validating the compile-time type against the handle's registered type and value layout |
| [`shape_bounds`](SHAPES.md) | procedure | Computes local bounds for a registered shape at an orientation |
| [`shape_ray`](SHAPES.md) | procedure | Tests one registered shape directly, without inserting it into a world or traversing the broad phase |
| [`shape_remove_recursive`](SHAPES.md) | procedure | Removes an unreferenced root and then removes every now-unreferenced reachable child shape |

### [compound_mass.odin](../../../src/entasis/compound_mass.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Compound_Builder`](SHAPES.md) | struct | Caller-owned zero-allocation view over child and mass arrays |
| [`Compound_Build_Result`](SHAPES.md) | struct | Contains a registered shape and mass properties |
| [`compound_builder`](SHAPES.md) | procedure | Binds caller-owned child and mass arrays without allocation |
| [`compound_center_of_mass`](SHAPES.md) | procedure | Computes the weighted center and inverse total mass without allocating or mutating children |
| [`compound_inertia_weighted`](SHAPES.md) | procedure | Computes mass properties around the current child origin |
| [`compound_inertia_weighted_recenter`](SHAPES.md) | procedure | Computes mass properties about the center of mass and subtracts that center from every caller-owned child position |
| [`compound_build_dynamic`](SHAPES.md) | procedure | Optionally recenters caller-owned children, computes weighted inertia, and registers a standard Compound in one owner-thread call |
| [`big_compound_build_dynamic`](SHAPES.md) | procedure | Performs the same weighted mass calculation and registers a tree-accelerated Big_Compound |

### [mesh_mass.odin](../../../src/entasis/mesh_mass.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Mesh_Mass_Properties`](SHAPES.md) | struct | Contains inertia and the center used by recentering operations. volume is signed for closed meshes and zero for open meshes |
| [`mesh_closed_inertia`](SHAPES.md) | procedure | Computes closed-solid inertia for a registered mesh |
| [`mesh_open_inertia`](SHAPES.md) | procedure | Computes triangle-soup inertia for a registered mesh |
| [`mesh_closed_center_of_mass`](SHAPES.md) | procedure | Returns signed closed volume and center of mass |
| [`mesh_open_center_of_mass`](SHAPES.md) | procedure | Returns the area-weighted center of an open triangle soup |

### [cooked_import.odin](../../../src/entasis/cooked_import.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Compound_Child`](SHAPES.md) | type alias | Exact low-level compound child representation |
| [`compound_child`](SHAPES.md) | procedure | Creates one compound child from a registered shape and local pose |
| [`shape_import_convex_hull`](SHAPES.md) | procedure | Copies a fully cooked hull into the world's pool and registers it without rebuilding hull topology |
| [`shape_import_mesh`](SHAPES.md) | procedure | Copies a fully cooked mesh, including its acceleration tree, into the world's pool and registers it without rebuilding the tree |
| [`shape_import_compound`](SHAPES.md) | procedure | Copies caller-owned child descriptors into the world's pool and registers one compound |
| [`shape_import_big_compound`](SHAPES.md) | procedure | Copies caller-owned child descriptors, builds the internal child tree in the world's pool, and registers one Big_Compound |
| [`shape_import_big_compound_cooked`](SHAPES.md) | procedure | Imports a prebuilt Big_Compound tree |

### [custom_shapes.odin](../../../src/entasis/custom_shapes.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`MAXIMUM_SHAPE_TYPE_COUNT`](SHAPES.md) | constant | Fixed shape type registry capacity |
| [`BUILT_IN_SHAPE_TYPE_COUNT`](SHAPES.md) | constant | Number of shape type IDs reserved by Entasis |
| [`MAXIMUM_COLLISION_TASK_COUNT`](SHAPES.md) | constant | Fixed collision task registry capacity |
| [`MAXIMUM_SWEEP_TASK_COUNT`](SHAPES.md) | constant | Fixed sweep task registry capacity |
| [`Shape_Batch_Type`](SHAPES.md) | type alias | Classifies custom shape storage and traversal behavior |
| [`Shape_Bounds`](SHAPES.md) | type alias | Stores a custom shape bounding box and angular expansion |
| [`Shape_Ray_Hit`](SHAPES.md) | type alias | Stores one ray hit against a custom shape |
| [`Shape_Registry`](SHAPES.md) | type alias | Low-level registry used by custom shape callbacks |
| [`Shape_Bounds_Proc`](SHAPES.md) | type alias | Computes bounds for one custom shape value |
| [`Shape_Inertia_Proc`](SHAPES.md) | type alias | Computes inertia for one custom shape value |
| [`Shape_Ray_Proc`](SHAPES.md) | type alias | Tests one ray against a custom shape value |
| [`Shape_Support_Proc`](SHAPES.md) | type alias | Returns a support point for one custom convex shape |
| [`Shape_Dispose_Proc`](SHAPES.md) | type alias | Releases caller-owned data stored by a custom shape |
| [`Collision_Task_Pair_Type`](SHAPES.md) | type alias | Selects direct or bounds-tested pair batching |
| [`Collision_Task_Kind`](SHAPES.md) | type alias | Classifies the manifold produced by a collision task |
| [`Collision_Task_Capability`](SHAPES.md) | type alias | Identifies one result form supported by a collision task |
| [`Collision_Task_Capabilities`](SHAPES.md) | type alias | Combines supported collision task result forms |
| [`Collision_Test_Proc`](SHAPES.md) | type alias | Tests one scalar pair for a convex manifold |
| [`Collision_Wide_Test_Proc`](SHAPES.md) | type alias | Tests one SIMD pair bundle |
| [`Collision_Wide_Manifold_Result`](SHAPES.md) | type alias | Stores SIMD manifold output |
| [`Collision_Convex_Wide_Bundle`](SHAPES.md) | type alias | Stores SIMD convex pair input |
| [`Sweep_Test_Proc`](SHAPES.md) | type alias | Sweeps one registered shape pair |
| [`Sweep_Child_Test_Proc`](SHAPES.md) | type alias | Sweeps one child pair inside a compound route |
| [`Custom_Shape_Registration`](SHAPES.md) | struct | Describes storage and callbacks for one custom shape type |
| [`Collision_Task_Registration`](SHAPES.md) | struct | Describes one custom collision pair route |
| [`Sweep_Task_Registration`](SHAPES.md) | struct | Describes one custom sweep pair route |
| [`custom_shape_registration`](SHAPES.md) | procedure | Infers storage size and alignment for one custom shape value type |
| [`custom_shape_next_type_id`](SHAPES.md) | procedure | Returns the sequential ID required by the registry |
| [`custom_shape_register`](SHAPES.md) | procedure | Installs one custom shape type |
| [`custom_shape_add`](SHAPES.md) | procedure | Stores one value under a previously registered custom type |
| [`collision_task_convex`](SHAPES.md) | procedure | Builds a convex custom collision task registration |
| [`collision_task_compound`](SHAPES.md) | procedure | Builds a bounds-tested compound collision task registration |
| [`collision_task_register`](SHAPES.md) | procedure | Installs one pair route |
| [`sweep_task_registration`](SHAPES.md) | procedure | Builds a custom sweep task registration |
| [`sweep_task_register`](SHAPES.md) | procedure | Installs one custom sweep route |

### [contextual_shapes.odin](../../../src/entasis/contextual_shapes.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Shape_Access`](SHAPES.md#contextual-shape-callbacks) | type alias | Borrowed read-only callback-local child access |
| [`Shape_Access_Value`](SHAPES.md#contextual-shape-callbacks) | type alias | Checked child payload view and layout metadata |
| [`Contextual_Shape_Bounds_Proc`](SHAPES.md#contextual-shape-callbacks) | procedure type | Contextual shape bounds callback |
| [`Contextual_Shape_Inertia_Proc`](SHAPES.md#contextual-shape-callbacks) | procedure type | Contextual inertia callback |
| [`Contextual_Shape_Ray_Proc`](SHAPES.md#contextual-shape-callbacks) | procedure type | Contextual ray callback |
| [`Contextual_Shape_Support_Proc`](SHAPES.md#contextual-shape-callbacks) | procedure type | Contextual support or sweep-support callback |
| [`Contextual_Shape_Dispose_Proc`](SHAPES.md#contextual-shape-callbacks) | procedure type | Contextual instance disposal callback |
| [`Contextual_Custom_Shape_Registration`](SHAPES.md#contextual-shape-callbacks) | structure | Shape payload layout and copied callback binding |
| [`custom_shape_registration_contextual`](SHAPES.md#contextual-shape-callbacks) | procedure | Infer payload layout and select default sweep/disposal behavior |
| [`custom_shape_register_contextual`](SHAPES.md#contextual-shape-callbacks) | procedure | Register one world-owned contextual shape type |
| [`shape_access_resolve`](SHAPES.md#contextual-shape-callbacks) | procedure alias | Resolve a live child without exposing mutation or scratch |
| [`shape_access_compute_bounds`](SHAPES.md#contextual-shape-callbacks) | procedure alias | Compute registered child bounds |
| [`shape_access_compute_inertia`](SHAPES.md#contextual-shape-callbacks) | procedure alias | Compute registered child inertia |
| [`shape_access_ray_test`](SHAPES.md#contextual-shape-callbacks) | procedure alias | Run the registered child ray callback |
| [`shape_access_support`](SHAPES.md#contextual-shape-callbacks) | procedure alias | Obtain the registered child support point |
| [`shape_access_sweep_support`](SHAPES.md#contextual-shape-callbacks) | procedure alias | Obtain the registered child sweep-support point |

### [contextual_tasks.odin](../../../src/entasis/contextual_tasks.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Contextual_Collision_Test_Proc`](SHAPES.md#contextual-collision-and-sweep-tasks) | procedure type | Scalar collision callback with explicit registration data |
| [`Contextual_Collision_Wide_Test_Proc`](SHAPES.md#contextual-collision-and-sweep-tasks) | procedure type | One collision callback per existing SIMD bundle |
| [`Contextual_Sweep_Test_Proc`](SHAPES.md#contextual-collision-and-sweep-tasks) | procedure type | Top-level sweep with separate registration and filter data |
| [`Contextual_Sweep_Child_Test_Proc`](SHAPES.md#contextual-collision-and-sweep-tasks) | procedure type | Registered child sweep with explicit registration data |
| [`Contextual_Collision_Task_Registration`](SHAPES.md#contextual-collision-and-sweep-tasks) | structure | Copied contextual scalar and wide collision descriptor |
| [`Contextual_Sweep_Task_Registration`](SHAPES.md#contextual-collision-and-sweep-tasks) | structure | Copied contextual top-level and child sweep descriptor |
| [`collision_task_register_contextual`](SHAPES.md#contextual-collision-and-sweep-tasks) | procedure | Register a world-owned contextual collision binding |
| [`sweep_task_register_contextual`](SHAPES.md#contextual-collision-and-sweep-tasks) | procedure | Register a world-owned contextual sweep binding |

## Constraints

Reference: [CONSTRAINTS.md](CONSTRAINTS.md)

### [constraints.odin](../../../src/entasis/constraints.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`CONSTRAINT_TYPE_INVALID`](CONSTRAINTS.md) | constant | Invalid public constraint type sentinel |
| [`Constraint_State`](CONSTRAINTS.md) | enum | Identifies whether a live constraint is stored in the active solver or in one sleeping island |
| [`Constraint_Type_ID`](CONSTRAINTS.md) | distinct type | Stable built-in constraint description identity |
| [`Servo_Settings`](CONSTRAINTS.md) | type alias | Correction-speed and force limits for position or orientation targets |
| [`Motor_Settings`](CONSTRAINTS.md) | type alias | Limits force and sets mass-scaled damping for velocity targets |
| [`Angular_Axis_Gear_Motor`](CONSTRAINTS.md) | type alias | Couples angular velocities along A's axis at a chosen ratio |
| [`Angular_Axis_Motor`](CONSTRAINTS.md) | type alias | Drives relative angular speed along A's local axis |
| [`Angular_Hinge`](CONSTRAINTS.md) | type alias | Aligns two local axes while leaving rotation around them free |
| [`Angular_Motor`](CONSTRAINTS.md) | type alias | Drives relative angular velocity expressed in A's local frame |
| [`Angular_Servo`](CONSTRAINTS.md) | type alias | Drives a target relative orientation with speed and force limits |
| [`Angular_Swivel_Hinge`](CONSTRAINTS.md) | type alias | Keeps the swivel and hinge axes perpendicular |
| [`Area_Constraint`](CONSTRAINTS.md) | type alias | Preserves the scaled triangle area between three body centers |
| [`Ball_Socket`](CONSTRAINTS.md) | type alias | Keeps two local anchor points together while allowing rotation |
| [`Ball_Socket_Motor`](CONSTRAINTS.md) | type alias | Drives B's anchor velocity relative to A in A's local frame |
| [`Ball_Socket_Servo`](CONSTRAINTS.md) | type alias | Brings two local anchors together with speed and force limits |
| [`Center_Distance_Constraint`](CONSTRAINTS.md) | type alias | Holds a target distance between body centers without local anchor offsets |
| [`Center_Distance_Limit`](CONSTRAINTS.md) | type alias | Bounds the distance between body centers |
| [`Distance_Limit`](CONSTRAINTS.md) | type alias | Bounds the distance between two local anchor points |
| [`Distance_Servo`](CONSTRAINTS.md) | type alias | Drives anchor separation to a target distance |
| [`Hinge`](CONSTRAINTS.md) | type alias | Joins local anchors and aligns axes while leaving hinge rotation free |
| [`Linear_Axis_Limit`](CONSTRAINTS.md) | type alias | Bounds anchor separation projected onto A's local axis |
| [`Linear_Axis_Motor`](CONSTRAINTS.md) | type alias | Drives relative anchor speed along A's local axis |
| [`Linear_Axis_Servo`](CONSTRAINTS.md) | type alias | Drives anchor separation along A's local plane normal |
| [`One_Body_Angular_Motor`](CONSTRAINTS.md) | type alias | Drives one body's world-space angular velocity |
| [`One_Body_Angular_Servo`](CONSTRAINTS.md) | type alias | Drives one body toward a world-space orientation |
| [`One_Body_Linear_Motor`](CONSTRAINTS.md) | type alias | Drives a local anchor's world-space velocity |
| [`One_Body_Linear_Servo`](CONSTRAINTS.md) | type alias | Drives a local anchor toward a world-space position |
| [`Point_On_Line_Servo`](CONSTRAINTS.md) | type alias | Keeps B's anchor on a line fixed in A's local frame |
| [`Swing_Limit`](CONSTRAINTS.md) | type alias | Bounds the angle between two body-local axes by their minimum dot |
| [`Swivel_Hinge`](CONSTRAINTS.md) | type alias | Joins anchors while keeping swivel and hinge axes perpendicular |
| [`Twist_Limit`](CONSTRAINTS.md) | type alias | Bounds relative twist between two local orientation bases |
| [`Twist_Motor`](CONSTRAINTS.md) | type alias | Drives relative twist speed around the two local axes |
| [`Twist_Servo`](CONSTRAINTS.md) | type alias | Drives a target twist angle between two local orientation bases |
| [`Volume_Constraint`](CONSTRAINTS.md) | type alias | Preserves scaled signed tetrahedron volume across four bodies |
| [`Weld`](CONSTRAINTS.md) | type alias | Holds the relative position and orientation of two bodies |
| [`Constraint_Info`](CONSTRAINTS.md) | struct | Compact pointer-free inspection record |
| [`servo_settings`](CONSTRAINTS.md) | procedure | Constructs the exact runtime servo settings |
| [`motor_settings`](CONSTRAINTS.md) | procedure | Constructs the existing high-stiffness motor settings. softness is zero for a rigid motor and increases as the motor becomes softer |
| [`motor_settings_damping`](CONSTRAINTS.md) | procedure | Constructs motor settings from the raw mass-scaled damping constant stored by the runtime |
| [`constraint_type_id`](CONSTRAINTS.md) | procedure | Returns the built-in public constraint type represented by T. Contact constraints and unsupported/custom descriptions return invalid |
| [`constraint_body_count`](CONSTRAINTS.md) | procedure | Returns the built-in description body arity or zero for an unsupported description type |
| [`constraint_add`](CONSTRAINTS.md) | procedure | Adds one built-in constraint from an exact-length body slice |
| [`constraint_add_1`](CONSTRAINTS.md) | procedure | Adds a one-body built-in constraint |
| [`constraint_add_2`](CONSTRAINTS.md) | procedure | Adds a two-body built-in constraint |
| [`constraint_add_3`](CONSTRAINTS.md) | procedure | Adds a three-body built-in constraint |
| [`constraint_add_4`](CONSTRAINTS.md) | procedure | Adds a four-body built-in constraint |
| [`constraint_get`](CONSTRAINTS.md) | procedure | Copies one active or sleeping built-in description into target |
| [`constraint_apply`](CONSTRAINTS.md) | procedure | Replaces one built-in description |
| [`constraint_remove`](CONSTRAINTS.md) | procedure | Removes one active or sleeping constraint |
| [`constraint_inspect`](CONSTRAINTS.md) | procedure | Returns a pointer-free body/type/storage snapshot without exposing the solver's AoSoA layout |
| [`constraint_count`](CONSTRAINTS.md) | procedure | Returns the total number of active and sleeping constraints |
| [`constraint_enumerate`](CONSTRAINTS.md) | procedure | Writes live constraints in ascending numeric handle order |
| [`constraint_add_batch_1`](CONSTRAINTS.md) | procedure | Adds homogeneous one-body constraints in input order |
| [`constraint_add_batch_2`](CONSTRAINTS.md) | procedure | Adds homogeneous two-body constraints in input order |
| [`constraint_add_batch_3`](CONSTRAINTS.md) | procedure | Adds homogeneous three-body constraints in input order |
| [`constraint_add_batch_4`](CONSTRAINTS.md) | procedure | Adds homogeneous four-body constraints in input order |
| [`constraint_apply_batch`](CONSTRAINTS.md) | procedure | Applies matching homogeneous descriptions in input order and returns the committed prefix |
| [`constraint_remove_batch`](CONSTRAINTS.md) | procedure | Removes constraints in input order and returns the committed prefix |

### [custom_constraints.odin](../../../src/entasis/custom_constraints.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`FIRST_CUSTOM_CONSTRAINT_TYPE_ID`](CONSTRAINTS.md) | constant | First type ID available to caller-defined constraints |
| [`MAXIMUM_CONSTRAINT_TYPE_COUNT`](CONSTRAINTS.md) | constant | Fixed constraint type registry capacity |
| [`MAXIMUM_CUSTOM_DESCRIPTION_BYTES`](CONSTRAINTS.md) | constant | Largest scalar custom constraint description |
| [`MAXIMUM_INACTIVE_IMPULSE_SCALARS`](CONSTRAINTS.md) | constant | Limits stored impulse scalars for inactive custom constraints |
| [`BODY_ACCESS_ALL`](CONSTRAINTS.md) | constant | Requests pose, inertia, and velocity data |
| [`BODY_ACCESS_NO_POSE`](CONSTRAINTS.md) | constant | Requests inertia and velocity without pose data |
| [`BODY_ACCESS_NO_POSITION`](CONSTRAINTS.md) | constant | Requests orientation, inertia, and velocity |
| [`BODY_ACCESS_NO_ORIENTATION`](CONSTRAINTS.md) | constant | Requests position, inertia, and velocity |
| [`BODY_ACCESS_ONLY_VELOCITY`](CONSTRAINTS.md) | constant | Requests linear and angular velocity only |
| [`BODY_ACCESS_ONLY_ANGULAR`](CONSTRAINTS.md) | constant | Requests orientation, inertia, and angular velocity |
| [`BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE`](CONSTRAINTS.md) | constant | Requests inertia and angular velocity only |
| [`BODY_ACCESS_ONLY_LINEAR`](CONSTRAINTS.md) | constant | Requests position, inverse mass, and linear velocity |
| [`Body_Access`](CONSTRAINTS.md) | type alias | Identifies one body field requested by a custom constraint kernel |
| [`Body_Access_Mask`](CONSTRAINTS.md) | type alias | Combines body fields requested by a custom constraint kernel |
| [`Constraint_Kernel_Phase`](CONSTRAINTS.md) | type alias | Identifies the custom kernel execution phase |
| [`Constraint_Kernel_Body_Wide`](CONSTRAINTS.md) | type alias | Stores one SIMD body bundle supplied to a custom kernel |
| [`Constraint_Kernel_Proc`](CONSTRAINTS.md) | type alias | SIMD solve callback for a custom constraint type |
| [`F32x8`](CONSTRAINTS.md) | type alias | Eight-lane f32 SIMD value used by custom constraint kernels |
| [`I32x8`](CONSTRAINTS.md) | type alias | Eight-lane i32 SIMD value used by custom constraint kernels |
| [`Vector2_Wide`](CONSTRAINTS.md) | type alias | Stores eight Vector2 values in structure-of-arrays form |
| [`Vector3_Wide`](CONSTRAINTS.md) | type alias | Stores eight Vector3 values in structure-of-arrays form |
| [`Quaternion_Wide`](CONSTRAINTS.md) | type alias | Stores eight Quaternion values in structure-of-arrays form |
| [`Symmetric3x3_Wide`](CONSTRAINTS.md) | type alias | Stores eight symmetric matrices in structure-of-arrays form |
| [`Constraint_Description_Validate_Proc`](CONSTRAINTS.md) | type alias | Validates one scalar custom constraint description |
| [`Custom_Constraint_Registration`](CONSTRAINTS.md) | struct | Describes one caller-defined constraint type and its SIMD kernels |
| [`custom_constraint_registration`](CONSTRAINTS.md) | procedure | Infers the scalar description, wide prestep, and accumulated-impulse storage sizes |
| [`custom_constraint_next_type_id`](CONSTRAINTS.md) | procedure | Returns the first unused caller constraint type ID |
| [`custom_constraint_register`](CONSTRAINTS.md) | procedure | Installs one caller-defined constraint type before simulation starts |
| [`custom_constraint_add`](CONSTRAINTS.md) | procedure | Creates one registered custom constraint from raw description bytes |
| [`custom_constraint_add_typed`](CONSTRAINTS.md) | procedure | Creates one registered custom constraint from a typed description |
| [`custom_constraint_get`](CONSTRAINTS.md) | procedure | Copies one custom constraint description into caller-owned storage |
| [`custom_constraint_get_typed`](CONSTRAINTS.md) | procedure | Returns one typed custom constraint description |
| [`custom_constraint_apply`](CONSTRAINTS.md) | procedure | Replaces one custom constraint description from raw bytes |
| [`custom_constraint_apply_typed`](CONSTRAINTS.md) | procedure | Replaces one custom constraint with a typed description |

### Joint reactions and breaking

Reference: [breakable joints](CONSTRAINTS.md#breakable-joints). Source: [`breakable_joints.odin`](../../../src/entasis/breakable_joints.odin)

`Joint_Break_Metric`, `Joint_Break_Metrics`, `Joint_Break_Limits`, `Joint_Reaction`, `Joint_Reaction_State` and `Joint_Break_Event` describe watch settings and copied results. `Joint_Reaction_Provider`, `Joint_Reaction_Provider_Input` and `Joint_Impulse_Wrench` define custom reaction conversion

`world_enable_joint_breaks`, `world_disable_joint_breaks`, `joint_break_reserve`, `constraint_set_break_limits`, `constraint_get_break_limits`, `constraint_clear_break_limits`, `constraint_reaction`, `constraint_set_reaction_provider`, `constraint_break_events_drain` and `constraint_break_events_discard` configure watches and deliver break events

`joint_break_stepper_update` returns `Joint_Break_Update_Result`, with `Joint_Break_Pending_Streams` and `Joint_Break_Pending_Stream` describing pending parent, part, contact and break batches

### [connectivity.odin](../../../src/entasis/connectivity.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`body_constraint_count`](CONSTRAINTS.md) | procedure | Returns the number of constraints connected to a body |
| [`body_constraints`](CONSTRAINTS.md) | procedure | Writes the body's connected constraint handles in the runtime body-list order |
| [`constraint_connected_bodies`](CONSTRAINTS.md) | procedure | Writes the body handles referenced by one active or sleeping constraint |
| [`body_connected_bodies`](CONSTRAINTS.md) | procedure | Writes one entry for every connected constraint/body edge, excluding the source body |

## Collision

Reference: [COLLISION.md](COLLISION.md)

### [filtering.odin](../../../src/entasis/filtering.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`MAX_COLLISION_LAYER_COUNT`](COLLISION.md) | constant | Number of public collision layers |
| [`LAYER_MASK_NONE`](COLLISION.md) | constant | Empty collision layer mask |
| [`LAYER_MASK_ALL`](COLLISION.md) | constant | Mask containing every public collision layer |
| [`Collision_Layer`](COLLISION.md) | distinct type | Compact layer index in the inclusive range [0, 63] |
| [`Layer_Mask`](COLLISION.md) | distinct type | Stores one bit per Collision_Layer |
| [`Collision_Property_Table`](COLLISION.md) | type alias | Generation-aware collidable property table specialized for collision filtering and materials |
| [`Layer_Matrix`](COLLISION.md) | struct | Stores a symmetric global layer-pair policy |
| [`Collision_Filter`](COLLISION.md) | struct | Combines a layer with the layers this collidable accepts |
| [`Collision_Properties`](COLLISION.md) | struct | One dense property-table record per collidable |
| [`Layer_Material_Policy`](COLLISION.md) | struct | Caller owned and must remain at a stable address while its world can step |
| [`Collision_Pair_Filter_Proc`](COLLISION.md) | procedure | Optionally applies engine-specific pair filtering after layer and mask checks |
| [`Collision_Child_Filter_Proc`](COLLISION.md) | procedure | Optionally filters compound children or mesh triangles after the parent pair has passed layer filtering |
| [`collision_layer`](COLLISION.md) | procedure | Validates and constructs one compact layer index |
| [`collision_layer_index`](COLLISION.md) | procedure | Returns the integer layer index |
| [`collision_layer_is_valid`](COLLISION.md) | procedure | Validates the compact layer value |
| [`layer_mask`](COLLISION.md) | procedure | Creates a mask containing one layer |
| [`layer_mask_none`](COLLISION.md) | procedure | Returns an empty layer mask |
| [`layer_mask_all`](COLLISION.md) | procedure | Returns a mask containing every public collision layer |
| [`layer_mask_add`](COLLISION.md) | procedure | Returns a copy with one layer enabled |
| [`layer_mask_remove`](COLLISION.md) | procedure | Returns a copy with one layer disabled |
| [`layer_mask_contains`](COLLISION.md) | procedure | Checks one layer bit |
| [`collision_filter`](COLLISION.md) | procedure | Constructs one layer and mask pair |
| [`collision_filter_default`](COLLISION.md) | procedure | Returns layer zero accepting every layer |
| [`collision_properties`](COLLISION.md) | procedure | Constructs one dense per-collidable policy record |
| [`collision_properties_default`](COLLISION.md) | procedure | Uses the default filter and fallback material |
| [`collision_properties_filter`](COLLISION.md) | procedure | Reconstructs the transient layer/mask view from the compact 16-byte property record |
| [`collision_properties_validate`](COLLISION.md) | procedure | Checks the stable record representation |
| [`layer_matrix_none`](COLLISION.md) | procedure | Rejects every layer pair |
| [`layer_matrix_all`](COLLISION.md) | procedure | Accepts every layer pair |
| [`layer_matrix_set`](COLLISION.md) | procedure | Updates both directions of one layer pair |
| [`layer_matrix_allow`](COLLISION.md) | procedure | Enables one symmetric layer pair |
| [`layer_matrix_deny`](COLLISION.md) | procedure | Disables one symmetric layer pair |
| [`layer_matrix_allows`](COLLISION.md) | procedure | Checks both matrix directions |
| [`collision_filter_allows`](COLLISION.md) | procedure | Applies both per-collidable masks and the optional global layer matrix |
| [`collision_property_init`](COLLISION.md) | procedure | Initializes the generation-aware body and static property spaces |
| [`collision_property_ensure_capacity`](COLLISION.md) | procedure | Grows both namespaces independently |
| [`collision_property_set_body`](COLLISION.md) | procedure | Creates or updates one body policy record |
| [`collision_property_get_body`](COLLISION.md) | procedure | Returns one body policy record |
| [`collision_property_set_static`](COLLISION.md) | procedure | Creates or updates one static policy record |
| [`collision_property_get_static`](COLLISION.md) | procedure | Returns one static policy record |
| [`collision_property_set`](COLLISION.md) | procedure | Creates or updates a body or static policy record |
| [`collision_property_set_batch`](COLLISION.md) | procedure | Applies records in exact input order |
| [`collision_property_get`](COLLISION.md) | procedure | Returns one body or static policy record |
| [`collision_property_remove_body`](COLLISION.md) | procedure | Invalidates one body policy lifetime |
| [`collision_property_remove_static`](COLLISION.md) | procedure | Invalidates one static policy lifetime |
| [`collision_property_remove`](COLLISION.md) | procedure | Invalidates one body or static policy record |
| [`collision_property_remove_batch`](COLLISION.md) | procedure | Invalidates records in exact input order |
| [`collision_property_clear`](COLLISION.md) | procedure | Invalidates every property while retaining capacity |
| [`collision_property_destroy`](COLLISION.md) | procedure | Releases caller-owned property storage |
| [`layer_material_policy`](COLLISION.md) | procedure | Constructs a complete allocation-free narrow-phase policy |
| [`narrow_policy_layers_materials`](COLLISION.md) | procedure | Builds the stable layer, mask, pair-filter, child-filter, and material-table adapter around caller-owned context |

### [materials.odin](../../../src/entasis/materials.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`MATERIAL_ID_INVALID`](COLLISION.md) | constant | Fallback-material sentinel value |
| [`Material_ID`](COLLISION.md) | distinct type | Indexes one caller-owned Material_Table |
| [`Material`](COLLISION.md) | struct | Stable per-collidable material description used by the layer-and-material narrow-phase policy |
| [`Material_Table`](COLLISION.md) | struct | Caller-owned immutable view while a world is stepping |
| [`Material_Combine_Proc`](COLLISION.md) | procedure | Optionally replaces material_combine_default |
| [`material_id`](COLLISION.md) | procedure | Converts a nonnegative table index into a Material_ID. The sentinel value is reserved for fallback-material selection |
| [`material_id_invalid`](COLLISION.md) | procedure | Returns the fallback-material sentinel |
| [`material_id_is_valid`](COLLISION.md) | procedure | Checks only the sentinel representation |
| [`material`](COLLISION.md) | procedure | Constructs one per-collidable material without allocation |
| [`material_default`](COLLISION.md) | procedure | Returns the material used by the default narrow phase when two default collidables interact |
| [`material_table`](COLLISION.md) | procedure | Binds caller-owned material storage |
| [`material_to_contact`](COLLISION.md) | procedure | Converts the facade material to the exact low-level contact material consumed by contact constraints |
| [`material_validate`](COLLISION.md) | procedure | Applies the same validation used by the contact constraint path |
| [`material_table_get`](COLLISION.md) | procedure | Performs one bounds check and returns a pointer into the caller-owned table |
| [`material_combine_default`](COLLISION.md) | procedure | Combine friction, recovery speed and spring settings without depending on pair order |

### [collision_queries.odin](../../../src/entasis/collision_queries.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Collision_Query`](COLLISION.md) | struct | Describes one direct registered-shape pair test |
| [`Collision_Query_Result`](COLLISION.md) | struct | Pointer-free direct manifold result |
| [`collision_query`](COLLISION.md) | procedure | Tests two registered shapes directly without inserting them into the world |
| [`collision_query_batch`](COLLISION.md) | procedure | Executes direct shape-pair tests in input order |

### [contact_events.odin](../../../src/entasis/contact_events.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Contact_Event_Kind`](COLLISION.md) | enum | Describes the pair transition observed after one completed world step |
| [`Contact_Event_Flag`](COLLISION.md) | enum | Describes which optional event fields are available |
| [`Contact_Event_Flags`](COLLISION.md) | bit set | Compact availability bitset for optional event fields |
| [`Contact_User_Table`](COLLISION.md) | type alias | Generation-aware property table used to translate collidable references into caller-selected stable IDs |
| [`Contact_Event`](COLLISION.md) | struct | One owner-thread pair transition |
| [`Contact_Tracker`](COLLISION.md) | struct | Caller-owned event state |
| [`contact_user_table_init`](COLLISION.md) | procedure | Initializes caller-owned body and static ID spaces |
| [`contact_user_table_ensure_capacity`](COLLISION.md) | procedure | Grows body and static user-ID capacities |
| [`contact_user_set_body`](COLLISION.md) | procedure | Sets one generation-aware body user ID |
| [`contact_user_set_static`](COLLISION.md) | procedure | Sets one generation-aware static user ID |
| [`contact_user_remove_body`](COLLISION.md) | procedure | Ends one body user-ID property lifetime |
| [`contact_user_remove_static`](COLLISION.md) | procedure | Ends one static user-ID property lifetime |
| [`contact_user_table_clear`](COLLISION.md) | procedure | Clears user-ID properties while retaining capacity |
| [`contact_user_table_destroy`](COLLISION.md) | procedure | Releases user-ID property storage |
| [`contact_tracker_init`](COLLISION.md) | procedure | Initializes caller-owned pair history. initial_capacity is the maximum simultaneously retained pair count before explicit growth |
| [`contact_tracker_capacity`](COLLISION.md) | procedure | Returns the current pair-history capacity |
| [`contact_tracker_ensure_capacity`](COLLISION.md) | procedure | Grows both pair-history buffers geometrically |
| [`contact_tracker_bind`](COLLISION.md) | procedure | Attaches a ready tracker to one ready world and optional caller-owned user-ID table |
| [`contact_tracker_clear`](COLLISION.md) | procedure | Discards all pair history while retaining allocation and binding |
| [`contact_tracker_unbind`](COLLISION.md) | procedure | Discards history and detaches the tracker from its world |
| [`contact_tracker_destroy`](COLLISION.md) | procedure | Releases pair-history storage |
| [`contact_events_drain`](COLLISION.md) | procedure | Snapshots the pair cache after exactly one successful world_step and writes Begin, Persist, and End events into caller-owned storage |
| [`contact_events_discard`](COLLISION.md) | procedure | Advances pair history after exactly one completed step without producing events |

### Trigger colliders

`Trigger_Selection` uses `.Disabled` / `.Enabled` for Stay and static/static sampling. `Trigger_Notification_State` uses `.Ready` / `.Pending` for stepper delivery

See [trigger behavior](COLLISION.md#trigger-colliders) and the [trigger facade source](../../../src/entasis/triggers.odin)

`world_enable_triggers`, `world_disable_triggers`, `trigger_configuration_default`, `trigger_reserve`, `trigger_set`, `trigger_get`, `trigger_remove`, `trigger_set_user_id`, `trigger_mark_geometry_dirty`, `trigger_mark_filters_dirty`, `trigger_reset_history`, `trigger_events_drain`, `trigger_events_discard`, `trigger_overlaps` and `trigger_stepper_update` configure triggers and deliver their events. `body_collidable_reference` and `static_collidable_reference` resolve ordinary instance handles into query/trigger references. Public value types are `Trigger_Configuration`, `Trigger_Settings`, `Trigger_Pair`, `Trigger_Event`, `Trigger_Event_Kind`, `Trigger_Exit_Reason` and `Trigger_Update_Result`

### Restitution

Source: [`restitution.odin`](../../../src/entasis/restitution.odin)

Usage: [restitution](COLLISION.md#restitution)

`Restitution_Settings`, `Restitution_Configuration`, `Restitution_Combine_Proc`,
`restitution_configuration_default`, `world_enable_restitution`,
`world_disable_restitution`, `restitution_reserve`, `restitution_set`,
`restitution_get`, and `restitution_remove` provide optional per-instance bounce
without changing existing materials or contact type IDs

### Mixed collider parts

Reference: [mixed solid and trigger parts](COLLISION.md#mixed-solid-and-trigger-parts). Source: [`mixed_colliders.odin`](../../../src/entasis/mixed_colliders.odin)

`Collider_Part_Role`, `Collider_Part_Option`, `Collider_Part_Options`, `Collider_Part_Settings`, `Collider_Part_Configuration`, `Collider_Part_Event_Subscription`, `Collider_Part_Identity` and `Collider_Part_Info` describe per-instance top-level parts. `Collider_Part_Endpoint`, `Collider_Part_Pair` and `Collider_Part_Event` carry copied event values

`collider_part_configuration_default`, `collider_parts_reserve`, `collider_part_set`, `collider_part_get`, `collider_part_remove` and `collider_parts` configure and inspect parts. `trigger_part_events_drain`, `trigger_part_events_discard` and `trigger_part_overlaps` expose the part stream

`collider_part_stepper_update` and `collider_part_stepper_update_with_contacts` return `Collider_Part_Update_Result` and `Collider_Part_Contact_Update_Result`. Their `Collider_Part_Pending_Streams` bit set uses `Collider_Part_Pending_Stream`

## Queries

Reference: [QUERIES.md](QUERIES.md)

### [queries.odin](../../../src/entasis/queries.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`COLLIDABLE_DYNAMIC`](QUERIES.md) | constant | Dynamic-body query targets |
| [`COLLIDABLE_KINEMATIC`](QUERIES.md) | constant | Kinematic-body query targets |
| [`COLLIDABLE_STATIC`](QUERIES.md) | constant | Static query targets |
| [`COLLIDABLE_BODIES`](QUERIES.md) | constant | Dynamic and kinematic body query targets |
| [`COLLIDABLE_ALL`](QUERIES.md) | constant | Includes all dynamic, kinematic, and static query targets |
| [`Query_Kind`](QUERIES.md) | enum | Identifies one caller-owned batch query |
| [`Collidable_Mask`](QUERIES.md) | bit set | Selects dynamic, kinematic, and static query targets |
| [`Sweep_Result`](QUERIES.md) | type alias | Alias for the low-level sweep result |
| [`Sweep_Hit_State`](QUERIES.md) | type alias | Classifies the impact state returned by the low-level sweep kernel |
| [`Sweep_Hit_Proc`](QUERIES.md) | type alias | Advanced synchronous sweep-hit callback |
| [`Sweep_Zero_Hit_Proc`](QUERIES.md) | type alias | Handles an overlap at sweep time zero |
| [`Sweep_Callbacks`](QUERIES.md) | type alias | Advanced low-level sweep callback table |
| [`Sweep_Collector`](QUERIES.md) | type alias | Advanced caller-owned collector bridge |
| [`Query_Filter`](QUERIES.md) | struct | Mobility masks and optional collidable/child callbacks |
| [`Ray_Hit`](QUERIES.md) | struct | Layout-compatible with the low-level collector hit |
| [`Sweep_Hit`](QUERIES.md) | struct | Layout-compatible with the low-level sweep collector hit |
| [`Overlap_Hit`](QUERIES.md) | struct | Includes the generated contact manifold for one collidable |
| [`Volume_Hit`](QUERIES.md) | struct | Identifies one collidable whose broad-phase bounds overlap the requested axis-aligned volume |
| [`Sweep_Settings`](QUERIES.md) | struct | Controls iterative sweep convergence |
| [`Query_Output`](QUERIES.md) | struct | Selects a caller-owned subrange in Query_Scratch |
| [`Query`](QUERIES.md) | struct | Caller-owned tagged query |
| [`Query_Result`](QUERIES.md) | struct | Written by query_batch in input order |
| [`Query_Scratch`](QUERIES.md) | struct | Caller-owned all-hit output arrays selected by each query's output range |
| [`Query_Allow_Proc`](QUERIES.md) | procedure | Optionally filters complete collidables after the mobility mask |
| [`Query_Allow_Child_Proc`](QUERIES.md) | procedure | Optionally filters compound children and mesh triangles |
| [`ray`](QUERIES.md) | procedure | Constructs a world-space ray. direction does not need to be normalized. T remains measured in units of direction length |
| [`query_filter_all`](QUERIES.md) | procedure | Returns the explicit unfiltered policy |
| [`query_filter_mobility`](QUERIES.md) | procedure | Selects one or more mobility classes without callbacks |
| [`collidable_mobility`](QUERIES.md) | procedure | Returns the packed reference mobility class |
| [`collidable_body_handle`](QUERIES.md) | procedure | Extracts a dynamic or kinematic body handle |
| [`collidable_static_handle`](QUERIES.md) | procedure | Extracts a static handle |
| [`ray_cast_any`](QUERIES.md) | procedure | Reports whether the ray intersects any permitted collidable |
| [`ray_cast_closest`](QUERIES.md) | procedure | Returns Not_Found when no permitted collidable is hit |
| [`ray_cast_all`](QUERIES.md) | procedure | Writes a successful traversal prefix into caller-owned storage |
| [`sweep_closest`](QUERIES.md) | procedure | Sweeps one registered shape through the world while treating world targets as stationary, matching Entasis sweep semantics |
| [`overlap_all`](QUERIES.md) | procedure | Performs exact shape overlap tests and writes contact manifolds to caller-owned storage in low-level traversal order |
| [`volume_all`](QUERIES.md) | procedure | Returns broad-phase collidables whose bounds overlap the volume in low-level traversal order |
| [`query_output`](QUERIES.md) | procedure | Selects a subrange in one Query_Scratch output array |
| [`query_ray_any`](QUERIES.md) | procedure | Constructs one batched any-ray-hit query |
| [`query_ray_closest`](QUERIES.md) | procedure | Constructs one batched closest-ray-hit query |
| [`query_ray_all`](QUERIES.md) | procedure | Constructs one batched all-ray-hit query |
| [`query_sweep_closest`](QUERIES.md) | procedure | Constructs one batched closest-sweep query |
| [`query_overlap_all`](QUERIES.md) | procedure | Constructs one batched overlap-all query |
| [`query_volume_all`](QUERIES.md) | procedure | Constructs one batched broad-phase volume query |
| [`query_scratch`](QUERIES.md) | procedure | Binds caller-owned ray, overlap, and volume hit arrays for query_batch |
| [`query_batch`](QUERIES.md) | procedure | Combines compatible pure ray work and writes one result per input in input order |
| [`sweep_query_advanced`](QUERIES.md) | procedure | Executes a sweep using a caller-owned low-level collector |
| [`sweep_any`](QUERIES.md) | procedure | Returns true on the first permitted impact and stops traversal |
| [`sweep_all`](QUERIES.md) | procedure | Writes every permitted impact into caller-owned storage in low-level traversal order |

### [query_context.odin](../../../src/entasis/query_context.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Query_Context`](QUERIES.md#independently-owned-query-contexts) | distinct type | Caller-owned opaque handle for one independently owned query workspace. One simultaneous caller |
| [`Query_Hit_State`](QUERIES.md#independently-owned-query-contexts) | enum | Explicit `.Miss` / `.Hit` result for context-based any-hit operations |
| [`Query_Context_Description`](QUERIES.md#independently-owned-query-contexts) | struct | Reservation hints for collision pairs, children and traversal storage |
| [`query_context_description_default`](QUERIES.md#independently-owned-query-contexts) | procedure | Returns the ordinary private-query reservation hints |
| [`query_context_init`](QUERIES.md#independently-owned-query-contexts) | procedure | Attaches a context to a live world and creates private scratch or exclusively borrows its pool |
| [`query_context_reserve`](QUERIES.md#independently-owned-query-contexts) | procedure | Reserves context-local scratch while the attached world is idle |
| [`query_context_destroy`](QUERIES.md#independently-owned-query-contexts) | procedure | Returns scratch to its owner, detaches the world and preserves a borrowed pool |
| [`ray_cast_any_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes ray cast any using caller-exclusive context scratch and the existing query semantics |
| [`ray_cast_closest_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes ray cast closest using caller-exclusive context scratch and the existing query semantics |
| [`ray_cast_all_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes ray cast all using caller-exclusive context scratch and the existing query semantics |
| [`sweep_any_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes sweep any using caller-exclusive context scratch and the existing query semantics |
| [`sweep_closest_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes sweep closest using caller-exclusive context scratch and the existing query semantics |
| [`sweep_all_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes sweep all using caller-exclusive context scratch and the existing query semantics |
| [`overlap_all_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes overlap all using caller-exclusive context scratch and the existing query semantics |
| [`volume_all_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes volume all using caller-exclusive context scratch and the existing query semantics |
| [`collision_query_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes collision query using caller-exclusive context scratch and the existing query semantics |
| [`collision_query_batch_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes collision query batch using caller-exclusive context scratch and the existing query semantics |
| [`query_batch_with_context`](QUERIES.md#independently-owned-query-contexts) | procedure | Executes query batch using caller-exclusive context scratch and the existing query semantics |
| [`query_batch_read_only_status`](QUERIES.md#independently-owned-query-contexts) | procedure | Check whether a closest-ray batch can run without shared scratch or callbacks |

### Distance and separation

Source: [`distance_queries.odin`](../../../src/entasis/distance_queries.odin)

Usage: [distance queries](QUERIES.md#closest-point-distance-and-separation)

`Distance_Query_Settings`, `Distance_Query_State`, `Shape_Distance_Result`,
`Shape_Correction_Result`, `Distance_Query_Capacity`, `Distance_Query_Kind`,
`Distance_Query` and `Distance_Query_Result` describe dedicated geometry requests

`distance_query_settings_default`, `distance_query_capacity_default` and
`distance_query_reserve` provide explicit setup. `shape_closest_point`,
`shape_distance`, `shape_penetration`, `shape_depenetrate`,
`distance_query_batch` and `collidable_shape_pose` provide on-demand results

Reservation and query procedures have matching `_with_context` variants

`Overlap_State`, `overlap_any` and `overlap_any_with_context` provide [early-out
geometric overlap](QUERIES.md#early-out-geometric-overlap). A successful miss is
`Separated, Ok`. Negative contact depths and task failures are not hits. `Distance_Query_Geometry` names the geometric result inside `Shape_Distance_Result`

### [forwarders.odin](../../../src/entasis/forwarders.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`ray_query`](QUERIES.md) | procedure | Traverse world geometry with an application ray collector |

## Data access

Reference: [DATA-ACCESS.md](DATA-ACCESS.md)

### [types.odin](../../../src/entasis/types.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`MAXIMUM_WORKER_COUNT`](DATA-ACCESS.md) | constant | Hard execution limit of the included solver and dispatcher |
| [`Status`](DATA-ACCESS.md) | type alias | Operation result code |
| [`Body_Handle`](DATA-ACCESS.md) | type alias | Live body handle. Numeric value may be reused after removal |
| [`Static_Handle`](DATA-ACCESS.md) | type alias | Live static handle. Numeric value may be reused after removal |
| [`Constraint_Handle`](DATA-ACCESS.md) | type alias | Live constraint handle. Numeric value may be reused after removal |
| [`Shape_Handle`](DATA-ACCESS.md) | type alias | Typed live shape handle. Registry slots may be reused after removal |
| [`Vector2`](DATA-ACCESS.md) | type alias | Two-component math vector |
| [`Vector3`](DATA-ACCESS.md) | type alias | Zero-copy math alias |
| [`Vector4`](DATA-ACCESS.md) | type alias | Zero-copy math alias |
| [`Quaternion`](DATA-ACCESS.md) | type alias | Zero-copy math alias |
| [`Matrix3x3`](DATA-ACCESS.md) | type alias | Zero-copy math alias |
| [`Symmetric3x3`](DATA-ACCESS.md) | type alias | Zero-copy inertia tensor alias |
| [`Bounding_Box`](DATA-ACCESS.md) | type alias | Zero-copy bounds alias |
| [`Rigid_Pose`](DATA-ACCESS.md) | type alias | Zero-copy pose alias |
| [`Body_Velocity`](DATA-ACCESS.md) | type alias | Zero-copy velocity alias |
| [`Body_Inertia`](DATA-ACCESS.md) | type alias | Zero-copy inertia alias |
| [`Body_Mobility`](DATA-ACCESS.md) | type alias | Body mobility classification |
| [`Motion_State`](DATA-ACCESS.md) | type alias | Stores a body pose and velocity |
| [`Body_Inertias`](DATA-ACCESS.md) | type alias | Stores local and world inertia for an active body |
| [`Body_Dynamics`](DATA-ACCESS.md) | type alias | Stores dense motion and inertia data for an active body |
| [`Body_Activity`](DATA-ACCESS.md) | type alias | Stores sleep state for an active body |
| [`Collidable`](DATA-ACCESS.md) | type alias | Stores shape, continuity, and broad-phase state for a body |
| [`Static_Record`](DATA-ACCESS.md) | type alias | Stores the low-level data for one static collidable |
| [`Activity_Description`](DATA-ACCESS.md) | type alias | Body sleeping thresholds |
| [`Collidable_Description`](DATA-ACCESS.md) | type alias | Describes shape, continuity, and speculative margins |
| [`Body_Description`](DATA-ACCESS.md) | type alias | Contains the complete plain-data description for creating or applying a body |
| [`Body_State`](DATA-ACCESS.md) | type alias | Pointer-free body snapshot with the exact body-description layout |
| [`Static_Description`](DATA-ACCESS.md) | type alias | Contains the complete plain-data description for creating or applying a static |
| [`Static_State`](DATA-ACCESS.md) | type alias | Pointer-free static snapshot with the exact static-description layout |
| [`Allocator`](DATA-ACCESS.md) | type alias | Odin allocator used by explicitly owned storage |
| [`Dispatcher`](DATA-ACCESS.md) | type alias | Low-level worker dispatch boundary used by the world |
| [`Buffer_Pool`](DATA-ACCESS.md) | type alias | Reusable unmanaged allocation pool used by Entasis |
| [`Collidable_Reference`](DATA-ACCESS.md) | type alias | Packed collidable identity |
| [`Continuous_Detection`](DATA-ACCESS.md) | type alias | CCD configuration value |
| [`Continuous_Detection_Mode`](DATA-ACCESS.md) | type alias | CCD mode enum |
| [`Ray`](DATA-ACCESS.md) | type alias | World-space ray used by the stable any, closest, and all-hit query helpers |
| [`Ray_Query_Hit`](DATA-ACCESS.md) | type alias | Stores one low-level tree ray hit |
| [`Ray_Query_Collector`](DATA-ACCESS.md) | type alias | Stores the ray-hit buffer, count, collection mode and callbacks |

### [views.odin](../../../src/entasis/views.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Active_Body_View`](DATA-ACCESS.md) | struct | Exposes the exact dense active-body columns |
| [`Active_Body_Row`](DATA-ACCESS.md) | struct | Zero-copy row assembled from the active columns |
| [`Static_View`](DATA-ACCESS.md) | struct | Exposes the exact dense static records and reverse handle map |
| [`Static_Row`](DATA-ACCESS.md) | struct | Zero-copy static row |
| [`active_body_view`](DATA-ACCESS.md) | procedure | Returns the dense active set without copying body data |
| [`body_view_valid`](DATA-ACCESS.md) | procedure | Reports whether a previously acquired active-body view still names the current active columns |
| [`active_body_row`](DATA-ACCESS.md) | procedure | Returns one zero-copy row |
| [`static_view`](DATA-ACCESS.md) | procedure | Returns dense static records without copying |
| [`static_view_valid`](DATA-ACCESS.md) | procedure | Reports whether a previously acquired static view still names the current dense static storage |
| [`static_view_row`](DATA-ACCESS.md) | procedure | Returns one zero-copy static row |

### [properties.odin](../../../src/entasis/properties.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Body_Property_Key`](DATA-ACCESS.md) | struct | Identifies one body-property lifetime |
| [`Static_Property_Key`](DATA-ACCESS.md) | struct | Identifies one static-property lifetime and rejects stale references after numeric static-handle reuse |
| [`Collidable_Property_Key`](DATA-ACCESS.md) | struct | Identifies one body or static property lifetime while preserving the separate body and static handle namespaces |
| [`Body_Property_Table`](DATA-ACCESS.md) | struct | Stores caller-owned dense values indexed by Body_Handle |
| [`Static_Property_Table`](DATA-ACCESS.md) | struct | Stores caller-owned dense values indexed by Static_Handle |
| [`Collidable_Property_Table`](DATA-ACCESS.md) | struct | Combines independent body and static tables so equal numeric body and static handles never alias the same property slot |
| [`body_property_init`](DATA-ACCESS.md) | procedure | Initializes a zero or previously destroyed table |
| [`body_property_ensure_capacity`](DATA-ACCESS.md) | procedure | Grows storage geometrically when required |
| [`body_property_capacity`](DATA-ACCESS.md) | procedure | Returns the current dense slot capacity, or zero when the table is not ready |
| [`body_property_set`](DATA-ACCESS.md) | procedure | Creates or updates one value |
| [`body_property_set_batch`](DATA-ACCESS.md) | procedure | Applies values in input order and returns the committed prefix before the first failure |
| [`body_property_get`](DATA-ACCESS.md) | procedure | Returns the current value for a handle |
| [`body_property_key`](DATA-ACCESS.md) | procedure | Captures a generation-protected key for the current entry |
| [`body_property_get_key`](DATA-ACCESS.md) | procedure | Resolves a key only while the same table and property lifetime remain active |
| [`body_property_remove`](DATA-ACCESS.md) | procedure | Clears one property and invalidates every key for its prior lifetime |
| [`body_property_remove_batch`](DATA-ACCESS.md) | procedure | Removes entries in input order and returns the removed prefix before the first missing or invalid handle |
| [`body_property_clear`](DATA-ACCESS.md) | procedure | Removes all entries, retains capacity, and invalidates all previously issued keys |
| [`body_property_destroy`](DATA-ACCESS.md) | procedure | Releases owned storage and invalidates all keys |
| [`static_property_init`](DATA-ACCESS.md) | procedure | Initializes a dense caller-owned static property table |
| [`static_property_ensure_capacity`](DATA-ACCESS.md) | procedure | Grows static property storage geometrically |
| [`static_property_capacity`](DATA-ACCESS.md) | procedure | Returns the current static slot capacity |
| [`static_property_set`](DATA-ACCESS.md) | procedure | Creates or updates one static value |
| [`static_property_set_batch`](DATA-ACCESS.md) | procedure | Applies values in input order and returns the committed prefix before the first failure |
| [`static_property_get`](DATA-ACCESS.md) | procedure | Returns one current value without allocation |
| [`static_property_key`](DATA-ACCESS.md) | procedure | Captures a generation-protected key for one entry |
| [`static_property_get_key`](DATA-ACCESS.md) | procedure | Rejects removed, cleared, destroyed, or reused lifetimes |
| [`static_property_remove`](DATA-ACCESS.md) | procedure | Clears one property and invalidates its prior keys |
| [`static_property_remove_batch`](DATA-ACCESS.md) | procedure | Removes entries in input order and returns the successful prefix |
| [`static_property_clear`](DATA-ACCESS.md) | procedure | Removes all entries while retaining capacity |
| [`static_property_destroy`](DATA-ACCESS.md) | procedure | Releases storage and permits later reinitialization |
| [`collidable_property_init`](DATA-ACCESS.md) | procedure | Initializes independent body and static property spaces using one allocator policy |
| [`collidable_property_ensure_capacity`](DATA-ACCESS.md) | procedure | Grows body and static spaces independently |
| [`collidable_property_set`](DATA-ACCESS.md) | procedure | Dispatches to the body or static namespace encoded in the collidable reference |
| [`collidable_property_get`](DATA-ACCESS.md) | procedure | Performs allocation-free dense lookup in the selected body or static namespace |
| [`collidable_property_key`](DATA-ACCESS.md) | procedure | Captures one generation-protected collidable property key |
| [`collidable_property_get_key`](DATA-ACCESS.md) | procedure | Validates table identity, epoch, and slot generation |
| [`collidable_property_remove`](DATA-ACCESS.md) | procedure | Clears the selected body or static property lifetime |
| [`collidable_property_clear`](DATA-ACCESS.md) | procedure | Clears both namespaces and retains their capacities |
| [`collidable_property_destroy`](DATA-ACCESS.md) | procedure | Releases both namespaces |
| [`collidable_property_set_batch`](DATA-ACCESS.md) | procedure | Applies body and static values in exact input order |
| [`collidable_property_remove_batch`](DATA-ACCESS.md) | procedure | Removes entries in exact input order and returns the successful prefix |

### [interop.odin](../../../src/entasis/interop.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Simulation`](DATA-ACCESS.md) | type alias | Advanced low-level simulation type |
| [`world_borrow_simulation`](DATA-ACCESS.md) | procedure | Returns the low-level simulation owned by world |
| [`world_borrow_pool`](DATA-ACCESS.md) | procedure | Returns the Buffer_Pool used by the world |
| [`world_borrow_dispatcher`](DATA-ACCESS.md) | procedure | Returns the blocking low-level dispatcher currently owned or referenced by world |

### [buffer_pool.odin](../../../src/entasis/buffer_pool.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`buffer_pool_init`](DATA-ACCESS.md) | procedure | Initializes a caller-owned pool for world_init_with_pool |
| [`buffer_pool_clear`](DATA-ACCESS.md) | procedure | Releases native blocks while retaining ready pool metadata |
| [`buffer_pool_destroy`](DATA-ACCESS.md) | procedure | Releases a caller-owned pool after all attached worlds are destroyed |

## Callbacks and stages

Reference: [CALLBACKS-STAGES.md](CALLBACKS-STAGES.md)

### [callbacks.odin](../../../src/entasis/callbacks.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Callback_Table_State`](CALLBACKS-STAGES.md) | enum | Distinguishes an empty callback table from a configured table |
| [`Contact_Material`](CALLBACKS-STAGES.md) | type alias | Zero-copy built-in contact material description |
| [`Spring_Settings`](CALLBACKS-STAGES.md) | type alias | Zero-copy built-in spring description used by contact materials and constraints |
| [`Narrow_Callbacks`](CALLBACKS-STAGES.md) | type alias | Raw narrow-phase callback table |
| [`Pose_Callbacks`](CALLBACKS-STAGES.md) | type alias | Advanced raw pose integration callback table |
| [`Collision_Testing_State`](CALLBACKS-STAGES.md) | type alias | Collision callback selection state |
| [`Angular_Integration_Mode`](CALLBACKS-STAGES.md) | type alias | Selects the angular velocity integration model |
| [`Velocity_Integration_State`](CALLBACKS-STAGES.md) | type alias | Identifies the current velocity integration pass |
| [`Manifold_Result`](CALLBACKS-STAGES.md) | type alias | Stores one narrow-phase manifold result |
| [`Convex_Contact_Manifold`](CALLBACKS-STAGES.md) | type alias | Stores contacts for one convex pair |
| [`Uniform_Gravity_Policy`](CALLBACKS-STAGES.md) | type alias | Runtime's uniform-gravity callback context. gravity, linear_damping, and angular_damping are caller-configured |
| [`Default_Narrow_Policy`](CALLBACKS-STAGES.md) | struct | Allows every collidable and child pair and assigns one constant material to every accepted manifold |
| [`Planetary_Gravity_Policy`](CALLBACKS-STAGES.md) | struct | Applies inverse-square radial gravity around a fixed center |
| [`Per_Body_Gravity_Policy`](CALLBACKS-STAGES.md) | struct | Gathers one caller-owned gravity vector per body handle |
| [`spring_settings`](CALLBACKS-STAGES.md) | procedure | Converts cycles per second and damping ratio into the exact spring representation used by the runtime |
| [`contact_material`](CALLBACKS-STAGES.md) | procedure | Constructs one constant built-in contact material |
| [`contact_material_default`](CALLBACKS-STAGES.md) | procedure | Returns the material used by the low-level default narrow-phase callbacks |
| [`default_narrow_policy`](CALLBACKS-STAGES.md) | procedure | Returns the no-filter, constant-material policy matching the low-level default |
| [`uniform_gravity_policy`](CALLBACKS-STAGES.md) | procedure | Returns a caller-owned context for the runtime's uniform-gravity and damping implementation |
| [`narrow_policy_default`](CALLBACKS-STAGES.md) | procedure | Builds a no-filter, constant-material callback table around caller-owned context |
| [`narrow_policy_default_stored_material`](CALLBACKS-STAGES.md) | procedure | Returns the material only when callbacks exactly match narrow_policy_default |
| [`pose_policy_uniform`](CALLBACKS-STAGES.md) | procedure | Builds the existing wide uniform-gravity callback table around caller-owned context |
| [`pose_policy_uniform_mode`](CALLBACKS-STAGES.md) | procedure | Builds the same uniform policy with an explicit angular integration mode |
| [`planetary_gravity_policy`](CALLBACKS-STAGES.md) | procedure | Creates a caller-owned inverse-square gravity policy |
| [`pose_policy_planetary`](CALLBACKS-STAGES.md) | procedure | Builds the wide radial-gravity callback table |
| [`per_body_gravity_policy`](CALLBACKS-STAGES.md) | procedure | Creates a caller-owned gravity gather policy around a Body_Property_Table(Vector3) |
| [`pose_policy_per_body`](CALLBACKS-STAGES.md) | procedure | Builds the wide per-body gravity callback table |
| [`world_description_set_callbacks`](CALLBACKS-STAGES.md) | procedure | Install copied callback tables with borrowed application context |

### [stages.odin](../../../src/entasis/stages.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Timestep_Completion_Stage`](CALLBACKS-STAGES.md) | type alias | Identifies stable completion points in the default timestepper |
| [`Stage_Completed_Proc`](CALLBACKS-STAGES.md) | type alias | Allocation-free contextless stage callback |
| [`Timestep_Callbacks`](CALLBACKS-STAGES.md) | type alias | Exact low-level stage callback table |
| [`Timestep_Proc`](CALLBACKS-STAGES.md) | type alias | Advanced complete-timestep callback ABI |
| [`Timestepper`](CALLBACKS-STAGES.md) | type alias | Selects a caller-owned complete timestep implementation |
| [`timestep_callbacks`](CALLBACKS-STAGES.md) | procedure | Constructs the stable stage-completion callback table |
| [`timestepper`](CALLBACKS-STAGES.md) | procedure | Constructs an advanced complete-timestep policy |
| [`world_description_set_timestep`](CALLBACKS-STAGES.md) | procedure | Installs optional caller-owned stage callbacks and a custom complete timestepper |
| [`world_stage_sleep`](CALLBACKS-STAGES.md) | procedure | Executes only sleeping/island migration |
| [`world_stage_predict_bounds`](CALLBACKS-STAGES.md) | procedure | Executes pose-policy preparation and predicted broad-phase leaf bounds for one timestep duration |
| [`world_stage_collision_detection`](CALLBACKS-STAGES.md) | procedure | Updates the broad phase and narrow phase |
| [`world_stage_solve`](CALLBACKS-STAGES.md) | procedure | Solves constraints and integrates bodies |
| [`world_stage_optimize`](CALLBACKS-STAGES.md) | procedure | Executes incremental solver/tree maintenance |

## Profiling and inspection

Reference: [PROFILING-INSPECTION.md](PROFILING-INSPECTION.md)

### [profiling.odin](../../../src/entasis/profiling.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Profile_Stage`](PROFILING-INSPECTION.md) | type alias | Identifies one low-level timestep stage |
| [`Profile_Snapshot`](PROFILING-INSPECTION.md) | struct | Pointer-free copy of the most recently completed step's stage timings plus cumulative stage invocation counts |
| [`World_Stats`](PROFILING-INSPECTION.md) | struct | Active and sleeping object counts and current step index |
| [`World_Solver_Stats`](PROFILING-INSPECTION.md) | struct | Pointer-free snapshot of active solver batch counts |
| [`profile_stage_text`](PROFILING-INSPECTION.md) | procedure | Returns one allocation-free static stage name |
| [`world_profile_enabled`](PROFILING-INSPECTION.md) | procedure | Reports whether profiling was enabled at world creation |
| [`world_profile_snapshot`](PROFILING-INSPECTION.md) | procedure | Copies the most recently completed profile |
| [`world_stats`](PROFILING-INSPECTION.md) | procedure | Returns stable read-only counts while the world is idle |
| [`world_solver_stats`](PROFILING-INSPECTION.md) | procedure | Returns read-only solver layout counts while the world is idle |

### [solver_inspection.odin](../../../src/entasis/solver_inspection.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`constraint_accumulated_impulses`](PROFILING-INSPECTION.md) | procedure | Writes all scalar warmstart impulses for one active or sleeping constraint |
| [`constraint_accumulated_impulse_magnitude_squared`](PROFILING-INSPECTION.md) | procedure | Returns the squared length of all scalar accumulated impulses associated with one constraint |
| [`constraint_accumulated_impulse_magnitude`](PROFILING-INSPECTION.md) | procedure | Returns the Euclidean length of all scalar accumulated impulses associated with one constraint |
| [`world_scale_active_accumulated_impulses`](PROFILING-INSPECTION.md) | procedure | Rescales active warmstart impulses |
| [`world_scale_accumulated_impulses`](PROFILING-INSPECTION.md) | procedure | Rescales active and sleeping warmstart impulses |

### [solver_contacts.odin](../../../src/entasis/solver_contacts.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`MAXIMUM_SOLVER_CONTACT_COUNT`](PROFILING-INSPECTION.md) | constant | Largest built-in manifold contact count |
| [`MAXIMUM_SOLVER_IMPULSE_COUNT`](PROFILING-INSPECTION.md) | constant | Bounds one pointer-free impulse snapshot |
| [`Contact_Constraint_Kind`](PROFILING-INSPECTION.md) | type alias | Identifies convex and nonconvex solver contact layouts |
| [`Solver_Contact_Point`](PROFILING-INSPECTION.md) | struct | One contact point reconstructed from solver prestep data, pair-cache feature identity, and accumulated impulse state |
| [`Solver_Contact_Data`](PROFILING-INSPECTION.md) | struct | Pointer-free snapshot of one active or sleeping built-in contact constraint |
| [`solver_contact_data`](PROFILING-INSPECTION.md) | procedure | Extracts one built-in contact constraint including contact offsets, depths, normals, feature IDs, material and accumulated impulses |

### [status.odin](../../../src/entasis/status.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`Diagnostic_Operation`](PROFILING-INSPECTION.md) | enum | Identifies a cold facade operation without storing or formatting a string |
| [`Diagnostic`](PROFILING-INSPECTION.md) | struct | Caller-owned status, operation and integer failure detail |
| [`status_ok`](PROFILING-INSPECTION.md) | procedure | Reports whether a public operation completed successfully |
| [`status_failed`](PROFILING-INSPECTION.md) | procedure | Reports whether a public operation failed |
| [`status_text`](PROFILING-INSPECTION.md) | procedure | Returns stable, allocation-free text for every public status |
| [`diagnostic_clear`](PROFILING-INSPECTION.md) | procedure | Resets optional caller-owned diagnostic context |
| [`diagnostic_record`](PROFILING-INSPECTION.md) | procedure | Stores allocation-free cold-path failure context |
| [`body_handle_invalid`](PROFILING-INSPECTION.md) | procedure | Returns the public invalid body-handle sentinel |
| [`body_handle_is_valid`](PROFILING-INSPECTION.md) | procedure | Checks only the sentinel representation |
| [`static_handle_invalid`](PROFILING-INSPECTION.md) | procedure | Returns the public invalid static-handle sentinel |
| [`static_handle_is_valid`](PROFILING-INSPECTION.md) | procedure | Checks only the sentinel representation |
| [`constraint_handle_invalid`](PROFILING-INSPECTION.md) | procedure | Returns the public invalid constraint-handle sentinel |
| [`constraint_handle_is_valid`](PROFILING-INSPECTION.md) | procedure | Checks only the sentinel representation |
| [`shape_handle_invalid`](PROFILING-INSPECTION.md) | procedure | Returns the public missing-shape sentinel |
| [`shape_handle_is_valid`](PROFILING-INSPECTION.md) | procedure | Checks only the typed-index existence bit |

### [version.odin](../../../src/entasis/version.odin)

| Declaration | Kind | Purpose |
| --- | --- | --- |
| [`VERSION_MAJOR`](PROFILING-INSPECTION.md) | constant | Product major version |
| [`VERSION_MINOR`](PROFILING-INSPECTION.md) | constant | Product minor version |
| [`VERSION_PATCH`](PROFILING-INSPECTION.md) | constant | Product patch version |
| [`VERSION_PRERELEASE`](PROFILING-INSPECTION.md) | constant | Empty for a stable release or contains the prerelease identifier |
| [`VERSION_STRING`](PROFILING-INSPECTION.md) | constant | Complete compile-time product version |
| [`ABI_VERSION`](PROFILING-INSPECTION.md) | constant | Identifies the public C ABI generation |
| [`CURRENT_VERSION`](PROFILING-INSPECTION.md) | constant | Contains the compile-time product version fields |
| [`Version`](PROFILING-INSPECTION.md) | struct | Compile-time Entasis product version |
| [`version_current`](PROFILING-INSPECTION.md) | procedure | Returns CURRENT_VERSION |
