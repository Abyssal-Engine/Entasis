# Constraints

Use constraints to connect bodies, limit relative motion, or drive motors and servos. Constraint descriptions define the participating bodies and solver settings

For every public declaration, see the [Constraints section of the API index](API-INDEX.md#constraints)

## Constraint lifecycle

A constraint belongs to one world and references one to four bodies according to its type

| Task                  | API                                                                            |
| --------------------- | ------------------------------------------------------------------------------ |
| Add with a body slice | `constraint_add`                                                               |
| Add with fixed arity  | `constraint_add_1`, `constraint_add_2`, `constraint_add_3`, `constraint_add_4` |
| Read description      | `constraint_get`                                                               |
| Replace description   | `constraint_apply`                                                             |
| Inspect metadata      | `constraint_inspect`                                                           |
| Remove                | `constraint_remove`                                                            |
| Count and enumerate   | `constraint_count`, `constraint_enumerate`                                     |

Body handles must be live in the same world. The description type and body count must match the registered constraint type

Removing a body removes every connected constraint. Removing a constraint makes its handle stale and the numeric slot may be reused

## Built-in constraint groups

### Point and distance

- `Ball_Socket`
- `Center_Distance_Constraint`
- `Center_Distance_Limit`
- `Distance_Limit`
- `Distance_Servo`
- `Point_On_Line_Servo`

### Angular relationships

- `Angular_Hinge`
- `Angular_Swivel_Hinge`
- `Hinge`
- `Swing_Limit`
- `Swivel_Hinge`
- `Twist_Limit`
- `Weld`

### Motors and servos

- `Angular_Axis_Gear_Motor`
- `Angular_Axis_Motor`
- `Angular_Motor`
- `Angular_Servo`
- `Ball_Socket_Motor`
- `Ball_Socket_Servo`
- `Linear_Axis_Motor`
- `Linear_Axis_Servo`
- `One_Body_Angular_Motor`
- `One_Body_Angular_Servo`
- `One_Body_Linear_Motor`
- `One_Body_Linear_Servo`
- `Twist_Motor`
- `Twist_Servo`

### Multi-body volume and area

- `Area_Constraint`
- `Volume_Constraint`

Use `constraint_type_id` and `constraint_body_count` when generic tooling needs to validate a description before submission

## Spring, servo, and motor settings

`spring_settings` is shared with contact materials and converts cycles per second plus damping ratio into the runtime spring representation

Use:

- `servo_settings` for maximum correction speed, base correction speed, and maximum force
- `motor_settings` for motor softness and maximum force
- `motor_settings_damping` when deriving motor settings from a damping value

These are plain-data description helpers and allocate nothing

## Batches

Fixed-arity add batches preserve input order:

- `constraint_add_batch_1`
- `constraint_add_batch_2`
- `constraint_add_batch_3`
- `constraint_add_batch_4`

`constraint_apply_batch` and `constraint_remove_batch` stop at the first failure and report how many entries succeeded. Those earlier entries remain applied. Retry only the remaining entries after correcting the failure

Caller-owned output handle storage must match the input count. Uncommitted output entries remain invalid

Constraint mutation invalidates direct body views because solver and body connectivity storage can change

## Connectivity

Use the connectivity APIs to inspect the live graph without parsing internal solver storage:

- `body_constraint_count`
- `body_constraints`
- `constraint_connected_bodies`
- `body_connected_bodies`

These return caller-owned graph snapshots while the world is idle, with explicit output capacity and no handle ownership transfer. Use them for graph traversal, sleeping groups or application cleanup

## Solver inspection

Accumulated impulse and reconstructed contact data are read-only snapshots of current solver state

Use [profiling and inspection](PROFILING-INSPECTION.md) for:

- Scalar accumulated impulse arrays
- Impulse magnitude helpers
- Active or all-set impulse scaling
- Built-in contact constraint reconstruction

## Custom constraints

Custom constraint registration defines:

- Stable custom type ID
- Body count
- Description byte size and validation
- Wide prestep and solve kernel callback
- Body access mask
- Sleeping impulse scalar count

Allocate an ID with `custom_constraint_next_type_id`, build a `Custom_Constraint_Registration`, and call `custom_constraint_register` before adding constraints or stepping the world

Use typed helpers when the application has a concrete description type:

- `custom_constraint_add_typed`
- `custom_constraint_get_typed`
- `custom_constraint_apply_typed`

The public registration API supports **8 custom constraint types per world**, using IDs 56-63. The other slots in the 64-entry registry are reserved for the engine. These are limits on distinct types, not on the number of constraint instances

A custom description can contain at most 256 bytes. Registration also limits its accumulated impulses to 32 scalar values so they can be retained when the constraint sleeps

The registered description layout must match the callback's expected packed `f32` representation and `F32x8` prestep access

`Body_Access_Mask` limits the motion and pose columns exposed to the kernel. Choose the narrowest mask that satisfies the implementation

[`custom_constraint`](../../../examples/headless/custom_constraint/main.odin) provides a complete custom registration and kernel

## Contextual constraints

Use `Contextual_Custom_Constraint_Registration` and `custom_constraint_register_contextual` when callbacks need borrowed application context. `custom_constraint_registration_contextual` infers packed-description, wide-prestep and impulse sizes and defaults the optional incremental callback to the main kernel. Direct descriptor construction supplies all three required callback pointers. Scalar and typed add/get/apply helpers operate on either registration kind

Follow [contextual registration lifetime](CALLBACKS-STAGES.md#contextual-registration-lifetime). Register before the first step or any constraint insertion, using the same type range, arity, packed-f32 and sleeping-impulse limits as native registration

The contextless validation callback receives user data, type ID and scalar description. The contextless kernel receives user data, pointer-based body-wide/prestep/impulse storage, timestep, an active-mask pointer and phase. All borrowed views expire at callback return. Follow the declared initial/solve access masks and preserve every inactive lane

## Threading and callbacks

Constraint lifecycle and enumeration are idle-world owner-thread operations

Custom kernels run inside solver work and must follow the registered wide layout. They cannot call owner-thread world mutation or retain temporary solver pointers after the callback returns

## Examples

The [example catalog](../EXAMPLES.md#body-motion-and-constraint-recipes) groups distance joints, motors, cloth, rope and ragdoll recipes. A [custom constraint example](../../../examples/headless/custom_constraint/main.odin) shows a wide kernel

## Body axis locks

Use the [body-control lock API](BODIES-STATICS.md#physical-axis-locks) for lock settings and impulse layout. Its one-body constraint uses engine-reserved type 48. Do not register that ID as a caller type

<a id="planned-extension"></a>

## Breakable joints

Enable optional watch storage with `world_enable_joint_breaks(world, watch_capacity)`, then call `constraint_set_break_limits` for selected constraints. `Joint_Break_Limits.metrics` selects `.Force`, `.Torque` or both. Selected limits must be finite and nonnegative. A load must be strictly greater than a selected limit to break the joint

Use these operations on the idle world owner thread. Pending publication or an undrained break batch blocks changes to limits/providers and disabling the feature

| Operation                         | Behavior                                                               |
| --------------------------------- | ---------------------------------------------------------------------- |
| `joint_break_reserve`             | Grow watch, selection and event capacity before stepping               |
| `constraint_set_break_limits`     | Create a watch or update its limits and copied `user_id`               |
| `constraint_get_break_limits`     | Read the current watch configuration                                   |
| `constraint_clear_break_limits`   | Remove the watch while retaining the joint                             |
| `constraint_reaction`             | Read the watched joint's last committed reaction                       |
| `constraint_break_events_drain`   | Complete pending publication and copy break events to caller storage   |
| `constraint_break_events_discard` | Complete publication and acknowledge events without copying            |
| `world_disable_joint_breaks`      | Release optional storage when no publication or event batch is pending |

Built-in noncontact joints, motors, servos and physical body-axis locks support typed reactions. Contact constraints are not breakable joints. A custom constraint needs a registered reaction provider before it can be watched. An empty metric set removes a watch, so `constraint_reaction` is not an unrestricted solver inspection call

Watches follow live constraint identity through solver relocation and sleeping. Removing a constraint or one of its bodies retires the watch. Reusing the numeric handle does not revive it. World clear removes watches and pending events. Setting a watch does not wake a sleeping island

### Typed joint reactions

`Joint_Reaction.sample.forces` contains world-space force and torque for each participating body, with torque measured about that body's center of mass. The engine reconstructs physical impulse wrenches and divides by the actual solved substep duration. Force has units of mass times length divided by time squared, and torque adds another length factor. A raw mixed linear/angular solver-impulse norm is not a substitute

`maximum_force` and `maximum_torque` retain independent peaks across completed substeps and participating bodies. `sample` is one representative substep chosen from the selected loads and limits, so its two magnitudes need not equal both independent peaks. `step` identifies the completed world step and `substep` is zero based. `state=.Unsolved` means the watch had no solved sample, including an entirely sleeping step

Reaction capture runs after joined solver work. It does not wake sleepers merely to read impulses. A failed step discards pending reaction publication and retains the prior committed result, but does not roll back physics already executed

### Custom reaction providers

After registering a custom constraint type and enabling joint breaks, use `constraint_set_reaction_provider` with a `Joint_Reaction_Provider`. The descriptor is copied and its `user_context` stays borrowed until replacement, disable or world destruction. A provider cannot be changed while any constraint of its type is watched

The synchronous owner-thread callback receives the type, participating poses, borrowed description, scalar accumulated impulses and actual substep duration. It returns one finite `Joint_Impulse_Wrench` per participating body, containing world-space linear and angular impulses about the center of mass. Return impulses, not already divided forces. Input pointers expire at callback return. Do not mutate or reenter the world, retain input views or unwind through the callback

A missing provider rejects watch admission. A provider failure or nonfinite output fails reaction capture and publishes no new break event for that failed step

### Removal and notification retry

Thresholds are evaluated per solved substep, but crossed joints remain in the solver until the complete world step succeeds. The completion phase awakens any crossed sleeping islands before removing joints and publishing ordered, value-copied `Joint_Break_Event` records. Each event carries the old handle, type, watch lifetime, copied user ID, exceeded metrics and reaction. The joint has already been removed when a published event is drained

An undrained nonempty break batch blocks another step. If output is too small, drain returns zero written and the required count. Retry with sufficient storage or discard. Do not resolve a delayed event solely through its numeric constraint handle, which may already be reused

Awakening can require allocation after physics completes. On failure, publication remains pending and no crossed joint is removed until all required awakening succeeds. Repair capacity and call drain, discard or the combined stepper to finish that same publication. These retries invalidate borrowed world views before attempting structural changes. Successful physics time must not be submitted again

With joint breaks enabled, a custom timestep returning `Ok` must advance the step index exactly once. An invalid success returns `Invalid_Argument` and discards pending feature publication without rolling back executed physics or the callback's step index. Manual solve stages alone do not publish a complete-step break batch

### Combined fixed-step delivery

`joint_break_stepper_update(stepper, world, elapsed, break_output, output, part_output, tracker, contact_output, dispatcher)` coordinates break, parent-trigger, part-trigger and optional contact streams. Joint breaks must be enabled. Parent delivery is selected whenever triggers are enabled, part delivery whenever its subscription is enabled, and contact delivery when a bound tracker is supplied. Empty output does not deselect an enabled stream

`Joint_Break_Update_Result` reports `completed_steps`, delivered counts for each stream, required capacities, `alpha` and a `pending` set containing `.Parent`, `.Part`, `.Contact` or `.Break`. Unselected streams require empty output. A supplied tracker must belong to the same world and cannot have skipped a completed step

The operation finishes pending break publication and delivers selected notifications before another physics step. Consume returned prefixes once, grow the indicated output or contact-history storage, and retry with `elapsed=0`. Required counts describe retained batches in addition to any delivered prefix. A contact prefix can be delivered before another stream reports backpressure. Notification retry does not replay completed physics
