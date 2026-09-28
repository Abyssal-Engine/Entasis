# Bodies and statics

Use dynamic bodies for solver-driven motion, kinematic bodies for prescribed motion, and statics for geometry without velocity or inertia

For every public declaration, see the [Bodies and statics section of the API index](API-INDEX.md#bodies-and-statics)

## Body descriptions

Use the facade constructors to build plain-data descriptions

| Constructor             | Result                                            |
| ----------------------- | ------------------------------------------------- |
| `pose`                  | `Rigid_Pose` with identity orientation by default |
| `velocity`              | Linear and angular velocity                       |
| `body_activity`         | Sleep threshold and required quiet timesteps      |
| `body_activity_default` | Ordinary facade sleep settings                    |
| `collidable`            | Shape, CCD mode, and speculative-margin settings  |
| `body_dynamic`          | Dynamic body with finite local inertia            |
| `body_kinematic`        | Kinematic body with locked inverse inertia        |
| `body_shapeless`        | Dynamic body with no broad-phase collidable       |

Descriptions and state copies contain no pointers into world storage. A body state is not a saved world: it does not include constraints, contacts or the rest of the simulation

## Body lifecycle

| Task                   | API           | Main rule                                                      |
| ---------------------- | ------------- | -------------------------------------------------------------- |
| Add                    | `body_add`    | May grow world storage                                         |
| Read                   | `body_get`    | Copies the state of an active or sleeping body |
| Replace complete state | `body_apply`  | Uses the full body mutation path                               |
| Remove                 | `body_remove` | Removes every connected constraint                             |

All body lifecycle operations require an idle owner-thread world

`body_remove` makes the handle stale. Its numeric value may later be reused, so remove external mappings and property entries after a successful physics removal

## Body setters

Use the narrowest setter that matches the change:

- `body_set_pose`
- `body_set_velocity`
- `body_set_inertia`
- `body_set_activity`
- `body_set_collidable`
- `body_set_shape`

`body_set_velocity` awakens a sleeping body. Shape, collidable, and inertia changes use the complete mutation path needed to keep broad-phase and solver state consistent

Every setter can invalidate direct active-body views

## Impulses and point velocity

| API                          | Behavior                                                           |
| ---------------------------- | ------------------------------------------------------------------ |
| `body_apply_linear_impulse`  | Changes linear velocity without explicitly awakening the body      |
| `body_apply_angular_impulse` | Uses the current world inverse inertia tensor                      |
| `body_apply_impulse`         | Applies an impulse at a world-space offset from the center of mass |
| `body_velocity_at_offset`    | Returns world-space point velocity at an offset                    |

Apply impulses only to a live body in an idle world. The impulse helpers do not hide a separate wake operation

[`impulses_solver_contacts`](../../../examples/headless/impulses_solver_contacts/main.odin) shows impulse application with contact and solver inspection

## Statics

Use `static_body` to build a `Static_Description`

| Task                       | API                     |
| -------------------------- | ----------------------- |
| Add                        | `static_add`            |
| Read                       | `static_get`            |
| Replace complete state     | `static_apply`          |
| Remove                     | `static_remove`         |
| Change pose                | `static_set_pose`       |
| Change shape               | `static_set_shape`      |
| Change continuity settings | `static_set_continuity` |

Every stable static mutation takes an `Awakening_Policy`

- `.Overlaps` wakes sleeping dynamic islands intersecting the old or new static bounds
- `.None` leaves sleeping islands asleep

The filtered static APIs accept `Static_Awakening_Filter_Proc` when the application must choose which overlap candidates wake

Statics do not carry body velocity or inertia

## Bounds

`body_bounds` and `static_bounds` return current broad-phase bounds

`body_update_bounds` and `static_update_bounds` exist for advanced direct pose writes. Call them after changing a borrowed pose outside the ordinary setter path so broad-phase data matches the new pose

Ordinary integrations should use `body_set_pose` and `static_set_pose`

## Sleeping

`Activity_Description` controls automatic sleep

A negative sleep threshold disables automatic sleeping. Otherwise a body must remain under the threshold for the configured number of consecutive timesteps before its connected island can sleep

Use:

- `body_activation_state`
- `body_is_active`
- `body_is_sleeping`
- `body_awaken`
- `bodies_sleep_group`

`body_awaken` wakes the complete constraint-connected island. An already active body is a successful no-op

`bodies_sleep_group` is an advanced owner-thread operation. Handles must be unique, active, and have no attached constraints

Sleeping migration and awakening invalidate direct active-body views

## Continuous collision detection

`Continuous_Detection` is stored in the body collidable description

| Builder          | Behavior                                                                              |
| ---------------- | ------------------------------------------------------------------------------------- |
| `ccd_discrete`   | No sweep and speculative expansion remains within configured margins                  |
| `ccd_passive`    | No sweep, but velocity-expanded bounds allow Continuous partners to discover the pair |
| `ccd_continuous` | Sweeps before speculative contact generation                                          |

Continuous mode uses `minimum_sweep_timestep` and `convergence_threshold`. Larger values reduce refinement work but can reduce sweep precision

Keep a positive maximum speculative margin for near-touching contacts. Setting it to zero can discard a sweep contact during contact generation

[`continuous_collision`](../../../examples/headless/continuous_collision/main.odin) compares continuous and discrete bodies

## Per-body gravity

Per-body gravity is installed through a pose policy and reads caller-owned `Body_Property_Table(Vector3)` storage

Keep the table synchronized with body lifetime and keep it valid for the world lifetime. See [callbacks and stages](CALLBACKS-STAGES.md#built-in-pose-policies) and [`per_body_gravity`](../../../examples/headless/per_body_gravity/main.odin)

## Optional body controls

1. Enable controls once with `world_enable_body_control`. The configuration is copied
2. While the world is idle, queue force/torque or a kinematic target and set any persistent damping or locks
3. Call `world_step` with the fixed physics duration
4. Submit new continuous inputs for the next step. Damping and locks persist until changed or cleared

Input capacity covers each queued-input and target table. Settings capacity covers damping. Both must be positive. Use `body_control_reserve` before exceeding capacity. Equal or smaller reservations reuse storage, and failed growth leaves live inputs and settings usable

Locks use ordinary solver capacity rather than these hints. Create them before a latency-critical step

Controls follow the world's allocator scope. `world_disable_body_control` removes owned axis-lock constraints and releases optional storage while preserving application joints

| Operation                                                                                 | Meaning                                                                                                       |
| ----------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `body_add_force`                                                                          | World force (`.Force`) or mass-independent linear acceleration (`.Acceleration`) for the next integrated step |
| `body_add_torque`                                                                         | World torque (`.Force`) or mass-independent angular acceleration (`.Acceleration`)                            |
| `body_add_force_at_position`                                                              | Freeze force and `(world_position - center_of_mass) x force` at submission                                    |
| `body_clear_inputs`                                                                       | Cancel queued continuous inputs. Not an undo of an executed step                                              |
| `body_set_damping` / `body_get_damping`                                                   | Set/query separate linear and angular exponential rates                                                       |
| `body_set_kinematic_target` / `body_get_kinematic_target` / `body_clear_kinematic_target` | Queue/query/cancel next-step kinematic motion                                                                 |

Use seconds and world coordinates. Force gives `delta_velocity = force * inverse_mass * integrated_dt`. Torque uses current world inverse inertia. These are not impulses: existing `body_apply_*_impulse` calls remain immediate and do not implicitly awaken. Repeated force submissions add. Zero inputs neither allocate nor wake. Non-finite inputs, unsupported modes and wrong mobility return `Invalid_Argument`

Force/target calls default to `.Wake`, which awakens the normal connected island. `.Preserve_Sleep` keeps an input pending until its body next integrates. Sleeping time does not accumulate duration. Removal, clear, mobility changes and handle reuse retire the affected state. Body-control mutations require owner-thread Ready state, not callback or concurrent-query access. Reacquire direct views after operations that may awaken bodies or alter target velocity

For an already initialized world and dynamic body, the following owner-thread helper queues one force and performs one step. Call `world_enable_body_control` once before using it. Do not enable on every frame

```odin
apply_thrust_step :: proc (world: ^entasis.World, body: entasis.Body_Handle, dt: f32) -> entasis.Status
{
    status := entasis.body_add_force(world, body, {20, 0, 0}, .Force, .Wake);
    if status != .Ok
    {
        return status;
    }
    return entasis.world_step(world, dt);
}
```

### Per-body damping

Damping rates have units of inverse seconds: the multiplier is `exp(-rate * integrated_dt)`, unlike the legacy global per-second fractional `Damping` settings. `.Additional` runs after the existing velocity callback and queued inputs, composing with authored damping

`.Override` replaces only the recognized native uniform, planetary or per-body gravity policy's damping while preserving its gravity calculation. Opaque custom callbacks reject Override. Zero Additional rates remove the setting. Absent settings read back as zero Additional rates. Damping changes do not awaken a body by themselves

### Step and target lifetime

Prediction uses inputs without consuming them. Actual integration applies them over the body's integrated duration, including its substeps, never once per solver iteration

Failed preparation preserves inputs. Once integration has applied an entry, a failed step consumes that entry and does not replay it on retry. Completed physics is not rolled back. Untouched sleeping inputs remain pending

Manual and custom stages use the same rule. Standalone predict, collision and solve must describe the same logical step duration. A full custom timestep supplies the full target duration even when it uses partial solve stages. Use the supplied simulation stages. Raw velocity writes do not record queued-input consumption

### Kinematic targets

Targets require kinematic bodies and normalized finite poses. They derive linear and shortest-arc angular velocities before contact detection, so moving kinematics interact through normal contacts. A target is not a teleport, stair/slide controller or guaranteed obstacle clearance. Substeps use the derived velocities. Only final position residuals of at most 1e-4 world units are snapped

Executed target motion expires even if later custom work fails. Its derived velocity is cleared before the next untargeted step unless an explicit caller body edit superseded it

Clearing a prepared target restores its previous velocity. Clearing an expired target clears only velocity still owned by that target. Replacing an expired target settles that owned velocity before storing its replacement, including after awakening. Cancelling the replacement cannot resurrect the expired velocity. Explicit caller velocity edits survive

### Physical axis locks

`Body_Axis_Lock` contains an explicit reference pose, `linear_axes` and `angular_axes` bit sets, and existing spring settings. Obtain defaults with `body_axis_lock_default(reference)`, then call `body_set_axis_lock`. Linear X/Y/Z rows hold world coordinates. Angular rows use fixed axes of the reference orientation and the existing shortest-arc angular-servo error, not Euler angles or axes that rotate with the body. The default spring is 30 Hz with damping ratio 1

These are iterative physical constraints: contacts, explicit joints and continuous inputs interact through their Jacobians and accumulated impulses. Accuracy depends on timestep, spring and solver iterations. They do not promise bit-exact immobility, post-solve clamping or arbitrary rotational limits. A zero inverse mass cannot receive linear correction. Degenerate zero-inertia axes cannot receive angular correction

With body controls already enabled on an idle world, this helper locks a dynamic body's current height and reference X/Z angular axes:

```odin
lock_current_height :: proc (world: ^entasis.World, body: entasis.Body_Handle) -> entasis.Status
{
    state, status := entasis.body_get(world, body);
    if status != .Ok
    {
        return status;
    }
    lock := entasis.body_axis_lock_default(state.pose);
    lock.linear_axes = {.Y};
    lock.angular_axes = {.X, .Z};
    return entasis.body_set_axis_lock(world, body, lock);
}
```

`body_set_axis_lock` copies and validates the complete value. Nonzero locks require a dynamic body. Changing a lock replaces its reference, masks and spring, and clears only its previous accumulated lock impulses. Explicit pose edits do not silently reanchor the lock: set a new reference to do so

A zero-axis descriptor or `body_clear_axis_lock` removes the lock. Clearing an absent lock on a live body succeeds. `body_get_axis_lock` returns `Not_Found` when absent and reads sleeping configuration without waking or allocating

Set, clear and control disable awaken connected sleeping islands through the existing solver and can need capacity. Failed insertion publishes no lock. It can leave the body awake and retain increased capacity

Removal, becoming kinematic, world clear and destruction retire the owned row. Handle reuse never inherits it. `bodies_sleep_group` is only for constraint-disconnected bodies and rejects locked/jointed bodies. Ordinary sleeping gathers the complete connected island. Disabling controls removes only the engine-owned rows, not explicit joints, and leaves the owner attached if awakening cannot complete. World destruction releases sleeping rows with the solver without awakening them merely for teardown

Axis locks are visible through constraint inspection and accumulated-impulse APIs. The six scalars represent world-linear X/Y/Z impulses followed by reference-angular X/Y/Z impulses. Unselected rows are zero. See [constraint inspection](CONSTRAINTS.md#body-axis-locks)
