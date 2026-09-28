package entasis

import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// MAXIMUM_WORKER_COUNT is the hard execution limit of the included solver and dispatcher
MAXIMUM_WORKER_COUNT :: physics.MAXIMUM_SOLVER_WORKER_COUNT;

// stable identity and result value types
Status            :: physics.Physics_Status;
// Body_Handle live body handle. numeric value may be reused after removal
Body_Handle       :: physics.Body_Handle;
// Static_Handle live static handle. numeric value may be reused after removal
Static_Handle     :: physics.Static_Handle;
// Constraint_Handle live constraint handle. numeric value may be reused after removal
Constraint_Handle :: physics.Constraint_Handle;
// Shape_Handle typed live shape handle. registry slots may be reused after removal
Shape_Handle      :: physics.Typed_Index;

// stable math and motion value types
Vector2      :: util.Vector2;
// Vector3 zero-copy math alias
Vector3      :: util.Vector3;
// Vector4 zero-copy math alias
Vector4      :: util.Vector4;
// Quaternion zero-copy math alias
Quaternion   :: util.Quaternion;
// Matrix3x3 zero-copy math alias
Matrix3x3    :: util.Matrix3x3;
// Symmetric3x3 zero-copy inertia tensor alias
Symmetric3x3 :: util.Symmetric3x3;
// Bounding_Box zero-copy bounds alias
Bounding_Box :: util.Bounding_Box;
// Rigid_Pose zero-copy pose alias
Rigid_Pose   :: physics.Rigid_Pose;
// Body_Velocity zero-copy velocity alias
Body_Velocity :: physics.Body_Velocity;
// Body_Inertia zero-copy inertia alias
Body_Inertia  :: physics.Body_Inertia;
// Body_Mobility body mobility classification
Body_Mobility :: physics.Body_Mobility;
// Motion_State stores a body pose and velocity
Motion_State  :: physics.Motion_State;
// Body_Inertias stores local and world inertia for an active body
Body_Inertias :: physics.Body_Inertias;
// Body_Dynamics stores dense motion and inertia data for an active body
Body_Dynamics :: physics.Body_Dynamics;
// Body_Activity stores sleep state for an active body
Body_Activity :: physics.Body_Activity;
// Collidable stores shape, continuity, and broad-phase state for a body
Collidable    :: physics.Collidable;
// Static_Record stores the low-level data for one static collidable
Static_Record :: physics.Static;

// stable plain-data creation descriptions and returned snapshots. state aliases
// preserve the exact low-level layout and perform no facade copy
Activity_Description   :: physics.Body_Activity_Description;
// Collidable_Description describes shape, continuity, and speculative margins
Collidable_Description :: physics.Collidable_Description;
// Body_Description contains the complete plain-data description for creating or applying a body
Body_Description       :: physics.Body_Description;
// Body_State is a pointer-free body snapshot with the exact body-description layout
Body_State             :: physics.Body_Description;
// Static_Description contains the complete plain-data description for creating or applying a static
Static_Description     :: physics.Static_Description;
// Static_State is a pointer-free static snapshot with the exact static-description layout
Static_State           :: physics.Static_Description;

// advanced explicit integration types
Allocator                 :: mem.Allocator;
// Dispatcher is the low-level worker dispatch boundary used by the world
Dispatcher                :: util.Thread_Dispatcher_Boundary;
// Buffer_Pool is the reusable unmanaged allocation pool used by Entasis
Buffer_Pool               :: util.Buffer_Pool;
// Collidable_Reference packed collidable identity
Collidable_Reference      :: physics.Collidable_Reference;
// Continuous_Detection is a CCD configuration value
Continuous_Detection      :: physics.Continuous_Detection;
// Continuous_Detection_Mode is the CCD mode enum
Continuous_Detection_Mode :: physics.Continuous_Detection_Mode;
// Ray is a world-space ray used by the stable any, closest, and all-hit query helpers
Ray                       :: physics.Tree_Ray;
// Ray_Query_Hit stores one low-level tree ray hit
Ray_Query_Hit             :: physics.Ray_Query_Hit;
// Ray_Query_Collector stores the ray-hit buffer, count, collection mode and callbacks
Ray_Query_Collector       :: physics.Ray_Query_Collector;
