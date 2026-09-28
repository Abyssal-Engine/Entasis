package benchmark_support

import "core:fmt"
import "core:os"
import "core:strings"
import "core:strconv"
import "core:math"
import entasis "entasis:entasis"

Workload :: enum u8
{
	Container,
	Contact_Islands,
	Pyramid,
}
Shape :: enum u8
{
	Box,
	Sphere,
	Capsule,
	Cylinder,
	Hull,
}
Sleep :: enum u8
{
	Disabled,
	Enabled,
}
Admission :: enum u8
{
	Ok,
	Invalid,
}
// these bounds allow i32 engine hints and exact handle/duration byte counts
MAX_BODIES :: 1_000_000;
MAX_STEPS :: 1_000_000;
PARAMETER_CAPACITY :: 8192;
Options :: struct
{
	workload: Workload,
	shape: Shape,
	static_shape: Shape,
	shape_size: [3]f32,
	density: f32,
	layout_scale: f32,
	steps: int,
	timestep_hz: int,
	warmup_steps: int,
	velocity_iterations: int,
	substeps: int,
	sleep: Sleep,
	grid: [3]int,
	spacing: [3]f32,
	spawn_height: f32,
	container_size: [3]f32,
	island_grid: [2]int,
	island_spacing: [2]f32,
	floor_size: [3]f32,
	rows: int,
	projectile_count: int,
	launch_step: int,
	projectile_radius: f32,
	projectile_density: f32,
	projectile_center: [3]f32,
	projectile_spacing: [3]f32,
	projectile_velocity: [3]f32,
	worker_count: int,
	output: string,
	recording: Recording_Options,
	population_count: int,
	body_count: int,
	static_count: int,
}

OPTION_NAMES :: [?]string{
	"shape",
	"static-shape",
	"shape-size",
	"density",
	"layout-scale",
	"steps",
	"timestep-hz",
	"warmup-steps",
	"velocity-iterations",
	"substeps",
	"sleep",
	"grid",
	"spacing",
	"spawn-height",
	"container-size",
	"island-grid",
	"island-spacing",
	"floor-size",
	"rows",
	"projectile-count",
	"launch-step",
	"projectile-radius",
	"projectile-density",
	"projectile-center",
	"projectile-spacing",
	"projectile-velocity",
	"worker-count",
	"output",
};

parse_integer :: proc(value: string) -> (int, Admission)
{
	if len(value) == 0
	{
		return 0, .Invalid;
	}
	result := 0;
	for c in value
	{
		if c < '0' || c > '9' || result > (int(max(i32)) - int(c - '0')) / 10
		{
			return 0, .Invalid;
		}
		result = result * 10 + int(c - '0');
	}
	return result, .Ok;
}

parse_number :: proc(value: string) -> (f32, Admission)
{
	// decimal grammar only. do not admit hex, underscores, whitespace or NaNs
	if len(value) == 0 || len(value) > 64
	{
		return 0, .Invalid;
	}
	for c in value
	{
		if !(c >= '0' && c <= '9') && c != '+' && c != '-' && c != '.' && c != 'e' && c != 'E'
		{
			return 0, .Invalid;
		}
	}
	number, ok := strconv.parse_f64(value);
	if !ok || !(number >= -f64(math.F32_MAX) && number <= f64(math.F32_MAX))
	{
		return 0, .Invalid;
	}
	if number == 0
	{
		return 0, .Ok;
	}
	converted := f32(number);
	if converted == 0
	{
		return 0, .Invalid;
	}
	return converted, .Ok;
}

parse_vector :: proc(value: string, target: []f32) -> Admission
{
	remaining := value;
	for &component, index in target
	{
		part, tail, found := split_once(remaining, ',');
		if (found == .Ok) != (index < len(target) - 1)
		{
			return .Invalid;
		}
		status: Admission;
		component, status = parse_number(part);
		if status != .Ok
		{
			return status;
		}
		remaining = tail;
	}
	return .Ok;
}

parse_grid :: proc(value: string, target: []int) -> Admission
{
	remaining := value;
	for &component, index in target
	{
		part, tail, found := split_once(remaining, ',');
		if (found == .Ok) != (index < len(target) - 1)
		{
			return .Invalid;
		}
		status: Admission;
		component, status = parse_integer(part);
		if status != .Ok || component <= 0
		{
			return .Invalid;
		}
		remaining = tail;
	}
	return .Ok;
}

parse_shape :: proc(value: string) -> (Shape, Admission)
{
	switch value
	{
		case "box": return .Box, .Ok;
		case "sphere": return .Sphere, .Ok;
		case "capsule": return .Capsule, .Ok;
		case "cylinder": return .Cylinder, .Ok;
		case "hull": return .Hull, .Ok;
	}
	return {}, .Invalid;
}

shape_volume :: proc(shape: Shape, size: [3]f32) -> f32
{
	x, y, z := size[0], size[1], size[2];
	r := x / 2;
	switch shape
	{
		case .Box, .Hull: return x * y * z;
		case .Sphere: return 4 / f32(3) * f32(math.PI) * r * r * r;
		case .Cylinder: return f32(math.PI) * r * r * y;
		case .Capsule: return f32(math.PI) * r * r * (y - x) + 4 / f32(3) * f32(math.PI) * r * r * r;
	}
	unreachable();
}

parse_workload_options :: proc(workload: Workload, arguments: []string) -> (Options, string)
{
	values: [len(OPTION_NAMES)]string;
	names := OPTION_NAMES;
	for argument in arguments
	{
		key, value, found := split_once(argument, '=');
		if found != .Ok || len(key) < 3 || key[:2] != "--" || len(value) == 0
		{
			return {}, argument;
		}
		option_index := -1;
		for name, index in OPTION_NAMES
		{
			if key[2:] == name
			{
				option_index = index;
				break;
			}
		}
		if option_index < 0 || len(values[option_index]) != 0
		{
			return {}, key;
		}
		values[option_index] = value;
	}
	o := Options{workload=workload, density=1, layout_scale=1, steps=300, timestep_hz=60,
		warmup_steps=30, velocity_iterations=4, substeps=1, grid={25, 16, 25},
		spacing={1.02, 1.02, 1.02}, spawn_height=24.51, container_size={29, 24.5, 29},
		island_grid={10, 5}, island_spacing={24, 24}, floor_size={12, 1, 12}, rows=36,
		projectile_count=4, launch_step=120, projectile_radius=2, projectile_density=10,
		projectile_center={-6, 2, 30}, projectile_spacing={4, 0, 0}, projectile_velocity={0, 0, -30}};
	if workload == .Contact_Islands
	{
		o.grid = {5, 8, 5};
		o.spacing = {0.995, 1.01, 0.995};
		o.spawn_height = 0.5;
	}
	if workload == .Pyramid
	{
		o.steps = 600;
		o.warmup_steps = 0;
		o.sleep = .Enabled;
		o.floor_size = {80, 1, 80};
	}
	status: Admission;
	if len(values[0]) == 0
	{
		return {}, "--shape is required";
	}
	o.shape, status = parse_shape(values[0]);
	if status != .Ok
	{
		return {}, "--shape";
	}
	if len(values[4]) > 0
	{
		o.layout_scale, status = parse_number(values[4]);
		if status != .Ok || o.layout_scale <= 0
		{
			return {}, "--layout-scale";
		}
	}
	scale := o.layout_scale;
	d := scale;
	if workload == .Pyramid
	{
		d *= 0.5;
	}
	o.shape_size = {d, d, d};
	if o.shape == .Capsule
	{
		o.shape_size = {d/2, d, d/2};
	}
	o.spacing *= scale;
	o.spawn_height *= scale;
	o.container_size *= scale;
	o.island_spacing *= scale;
	o.floor_size *= scale;
	o.projectile_radius *= scale;
	o.projectile_center *= scale;
	o.projectile_spacing *= scale;
	for value, index in values
	{
		if len(value) == 0
		{
			continue;
		}
		status = .Ok;
		switch index
		{
			case 0:
			o.shape, status = parse_shape(value);
			case 1:
			o.static_shape, status = parse_shape(value);
			case 2:
			status = parse_vector(value, o.shape_size[:]);
			case 3:
			o.density, status = parse_number(value);
			case 4:
			o.layout_scale, status = parse_number(value);
			case 5:
			o.steps, status = parse_integer(value);
			case 6:
			o.timestep_hz, status = parse_integer(value);
			case 7:
			o.warmup_steps, status = parse_integer(value);
			case 8:
			o.velocity_iterations, status = parse_integer(value);
			case 9:
			o.substeps, status = parse_integer(value);
			case 10:
			switch value
			{
				case "enabled": o.sleep = .Enabled;
				case "disabled": o.sleep = .Disabled;
				case: status = .Invalid;
			}
			case 11:
			if workload == .Pyramid
			{
				return {}, names[index];
			}
			status = parse_grid(value, o.grid[:]);
			case 12:
			if workload == .Pyramid
			{
				return {}, names[index];
			}
			status = parse_vector(value, o.spacing[:]);
			case 13:
			if workload == .Pyramid
			{
				return {}, names[index];
			}
			o.spawn_height, status = parse_number(value);
			case 14:
			if workload != .Container
			{
				return {}, names[index];
			}
			status = parse_vector(value, o.container_size[:]);
			case 15:
			if workload != .Contact_Islands
			{
				return {}, names[index];
			}
			status = parse_grid(value, o.island_grid[:]);
			case 16:
			if workload != .Contact_Islands
			{
				return {}, names[index];
			}
			status = parse_vector(value, o.island_spacing[:]);
			case 17:
			if workload == .Container
			{
				return {}, names[index];
			}
			status = parse_vector(value, o.floor_size[:]);
			case 18:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			o.rows, status = parse_integer(value);
			case 19:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			o.projectile_count, status = parse_integer(value);
			case 20:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			o.launch_step, status = parse_integer(value);
			case 21:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			o.projectile_radius, status = parse_number(value);
			case 22:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			o.projectile_density, status = parse_number(value);
			case 23:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			status = parse_vector(value, o.projectile_center[:]);
			case 24:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			status = parse_vector(value, o.projectile_spacing[:]);
			case 25:
			if workload != .Pyramid
			{
				return {}, names[index];
			}
			status = parse_vector(value, o.projectile_velocity[:]);
			case 26:
			o.worker_count, status = parse_integer(value);
			case 27:
			o.output = value;
		}
		if status != .Ok
		{
			return {}, names[index];
		}
	}
	if o.static_shape != .Box && o.static_shape != .Hull
	{
		return {}, "--static-shape";
	}
	for component in o.shape_size
	{
		if component <= 0 || finite_number(component) != .Ok
		{
			return {}, "--shape-size must be positive and finite";
		}
	}
	if (o.shape == .Sphere && o.shape_size[0] != o.shape_size[1]) ||
	(o.shape in bit_set[Shape]{.Sphere, .Cylinder, .Capsule} && o.shape_size[0] != o.shape_size[2]) ||
	(o.shape == .Capsule && o.shape_size[1] <= o.shape_size[0])
	{
		return {}, "--shape-size incompatible with --shape";
	}
	if o.density <= 0 || finite_number(o.density) != .Ok
	{
		return {}, "--density must be positive and finite";
	}
	if o.layout_scale <= 0 || finite_number(o.layout_scale) != .Ok
	{
		return {}, "--layout-scale must be positive and finite";
	}
	if workload == .Pyramid && (o.projectile_radius <= 0 || finite_number(o.projectile_radius) != .Ok)
	{
		return {}, "--projectile-radius must be positive and finite";
	}
	if workload == .Pyramid && (o.projectile_density <= 0 || finite_number(o.projectile_density) != .Ok)
	{
		return {}, "--projectile-density must be positive and finite";
	}
	if o.steps <= 0 || o.steps > MAX_STEPS
	{
		return {}, "--steps (1..1000000)";
	}
	if o.timestep_hz <= 0 || o.timestep_hz > MAX_STEPS
	{
		return {}, "--timestep-hz (1..1000000)";
	}
	if o.velocity_iterations <= 0 || o.velocity_iterations > MAX_STEPS
	{
		return {}, "--velocity-iterations (1..1000000)";
	}
	if o.substeps <= 0 || o.substeps > MAX_STEPS
	{
		return {}, "--substeps (1..1000000)";
	}
	if workload != .Pyramid
	{
		for v in o.spacing
		{
			if v <= 0 || finite_number(v) != .Ok
			{
				return {}, "--spacing must be positive and finite";
			}
		}
	}
	if workload == .Container
	{
		// boxes and cooked cuboid points both use f32 half-extents
		if entasis.shape_validate(entasis.box(o.container_size[0], o.container_size[1], o.container_size[2])) != .Ok
		{
			return {}, "--container-size must have positive finite half-extents";
		}
		if o.layout_scale / 2 <= 0
		{
			return {}, "--layout-scale produces zero container half-thickness";
		}
	}
	if workload != .Container
	{
		if entasis.shape_validate(entasis.box(o.floor_size[0], o.floor_size[1], o.floor_size[2])) != .Ok
		{
			return {}, "--floor-size must have positive finite half-extents";
		}
	}
	if workload == .Contact_Islands
	{
		for v in o.island_spacing
		{
			if v <= 0 || finite_number(v) != .Ok
			{
				return {}, "--island-spacing must be positive and finite";
			}
		}
	}
	if o.warmup_steps > MAX_STEPS || (workload == .Pyramid && o.warmup_steps != 0)
	{
		return {}, "--warmup-steps";
	}
	if o.worker_count < 1 || o.worker_count > 64 || len(o.output) == 0
	{
		return {}, "--worker-count (1..64) and --output are required";
	}
	if workload == .Pyramid
	{
		if o.rows <= 0 || o.rows > 144 || o.projectile_count > MAX_BODIES
		{
			return {}, "--rows / --projectile-count exceeds population limit";
		}
		o.population_count = o.rows * (o.rows + 1) * (2 * o.rows + 1) / 6;
		o.body_count = o.population_count + o.projectile_count;
		o.static_count = 1;
		if o.projectile_count > 0 && o.launch_step >= o.steps
		{
			return {}, "--launch-step must be less than --steps";
		}
	}
	else
	{
		count := 1;
		for n in o.grid
		{
			if n > MAX_BODIES / count
			{
				return {}, "--grid exceeds population limit";
			}
			count *= n;
		}
		o.static_count = 5;
		if workload == .Contact_Islands
		{
			for n in o.island_grid
			{
				if n > MAX_BODIES / count
				{
					return {}, "--island-grid exceeds population limit";
				}
				count *= n;
			}
			o.static_count = o.island_grid[0] * o.island_grid[1];
			if (o.island_grid[0] > 1 && o.island_spacing[0] < o.floor_size[0]) ||
			(o.island_grid[1] > 1 && o.island_spacing[1] < o.floor_size[2])
			{
				return {}, "--island-spacing overlaps --floor-size";
			}
		}
		o.population_count = count;
		o.body_count = count;
	}
	if o.body_count > MAX_BODIES
	{
		return {}, "population exceeds 1000000 bodies";
	}
 if inertia_admission(o.shape, o.shape_size, o.density) != .Ok
 {
  return {}, "--shape-size / --density produces unrepresentable inertia";
 }
 if workload == .Pyramid && o.projectile_count > 0 &&
  inertia_admission(.Sphere, {2*o.projectile_radius, 2*o.projectile_radius, 2*o.projectile_radius}, o.projectile_density) != .Ok
 {
  return {}, "--projectile-radius / --projectile-density produces unrepresentable inertia";
 }
 // check all derived placement extents with f64 arithmetic before f32 construction
 radius := math.sqrt(f64(o.shape_size[0])*f64(o.shape_size[0])+f64(o.shape_size[1])*f64(o.shape_size[1])+f64(o.shape_size[2])*f64(o.shape_size[2]))/2;
 if workload == .Pyramid
 {
  for axis in 0 ..< 3
  {
   if o.projectile_count > 0
   {
    extent := math.abs(f64(o.projectile_center[axis])) + f64(o.projectile_count-1)*math.abs(f64(o.projectile_spacing[axis])) + f64(o.projectile_radius);
    if !(extent <= f64(math.F32_MAX))
    {
     return {}, "--projectile-center / --projectile-spacing overflows placement";
    }
   }
  }
  if f64(o.rows)*f64(o.shape_size[1])+radius > f64(math.F32_MAX) || f64(o.rows)*f64(o.layout_scale)+radius > f64(math.F32_MAX)
  {
   return {}, "--rows / --shape-size / --layout-scale overflows placement";
  }
 }
 else
 {
  for axis in 0 ..< 3
  {
   extent := f64(o.grid[axis]-1)*f64(o.spacing[axis])+radius;
   if axis == 1
   {
    extent += math.abs(f64(o.spawn_height));
   }
   else if workload == .Contact_Islands
   {
    island_axis := axis/2;
    extent += f64(o.island_grid[island_axis]-1)*f64(o.island_spacing[island_axis]);
   }
   if !(extent <= f64(math.F32_MAX))
   {
    return {}, "--grid / --spacing / --spawn-height / --island-spacing overflows placement";
   }
  }
 }
	return o, "";
}

canonical_parameters :: proc(o: Options, buffer: []byte) -> (string, Admission)
{
	// at most 28 typed fields, nine-digit f32 components and fixed policy text.
	// 8192 bytes exceeds the 4096-byte worst case. caller owns the returned text
	if len(buffer) < PARAMETER_CAPACITY
	{
		return "", .Invalid;
	}
	shape_names := [Shape]string{.Box="box", .Sphere="sphere", .Capsule="capsule", .Cylinder="cylinder", .Hull="hull"};
	workload_names := [Workload]string{.Container="container", .Contact_Islands="contact_islands", .Pyramid="pyramid"};
	sleep_names := [Sleep]string{.Disabled="disabled", .Enabled="enabled"};
	lifecycle := "same-world;timing=per-step";
	if o.workload == .Container
	{
		lifecycle = "fresh-world-same-pool;timing=native-step-sum" if o.recording.mode == .On else "fresh-world-same-pool;timing=whole-loop";
	}
	if o.workload == .Pyramid
	{
		lifecycle = "no-warmup;timing=per-step-including-launch";
	}
	sleep_threshold := f32(-1);
	if o.sleep == .Enabled
	{
		sleep_threshold = 0.01;
	}
	result := fmt.bprintf(buffer, "workload=%s;shape=%s;static-shape=%s;shape-size=%.9g,%.9g,%.9g;density=%.9g;layout-scale=%.9g;steps=%d;timestep-hz=%d;warmup-steps=%d;velocity-iterations=%d;substeps=%d;sleep=%s;population=%d;bodies=%d;statics=%d;lifecycle=%s;gravity=0,-10,0;damping=0,0;material=0.5,2,30,1;ccd=passive;execution=fast;fallback=64;sleep-threshold=%.9g;sleep-minimum=32",
	workload_names[o.workload], shape_names[o.shape], shape_names[o.static_shape],
	o.shape_size[0], o.shape_size[1], o.shape_size[2], o.density, o.layout_scale, o.steps, o.timestep_hz,
	o.warmup_steps, o.velocity_iterations, o.substeps, sleep_names[o.sleep], o.population_count, o.body_count, o.static_count, lifecycle, sleep_threshold);
	offset := len(result);
	switch o.workload
	{
		case .Container, .Contact_Islands:
		result = fmt.bprintf(buffer[offset:], ";grid=%d,%d,%d;spacing=%.9g,%.9g,%.9g;spawn-height=%.9g",
		o.grid[0], o.grid[1], o.grid[2], o.spacing[0], o.spacing[1], o.spacing[2], o.spawn_height);
		offset += len(result);
		if o.workload == .Container
		{
			result = fmt.bprintf(buffer[offset:], ";container-size=%.9g,%.9g,%.9g;thickness=%.9g;maximum-speculative-margin=3.40282347e38",
			o.container_size[0], o.container_size[1], o.container_size[2], o.layout_scale);
		}
		else
		{
			result = fmt.bprintf(buffer[offset:], ";island-grid=%d,%d;island-spacing=%.9g,%.9g;floor-size=%.9g,%.9g,%.9g;maximum-speculative-margin=3.40282347e38",
			o.island_grid[0], o.island_grid[1], o.island_spacing[0], o.island_spacing[1], o.floor_size[0], o.floor_size[1], o.floor_size[2]);
		}
		case .Pyramid:
		result = fmt.bprintf(buffer[offset:], ";rows=%d;horizontal-spacing=%.9g;vertical-spacing=%.9g;floor-size=%.9g,%.9g,%.9g;projectile-count=%d;launch-step=%d;projectile-radius=%.9g;projectile-density=%.9g;projectile-center=%.9g,%.9g,%.9g;projectile-spacing=%.9g,%.9g,%.9g;projectile-velocity=%.9g,%.9g,%.9g;maximum-speculative-margin=3.40282347e38",
		o.rows, 0.5*o.layout_scale, o.shape_size[1], o.floor_size[0], o.floor_size[1], o.floor_size[2], o.projectile_count, o.launch_step, o.projectile_radius, o.projectile_density,
		o.projectile_center[0], o.projectile_center[1], o.projectile_center[2], o.projectile_spacing[0], o.projectile_spacing[1], o.projectile_spacing[2],
		o.projectile_velocity[0], o.projectile_velocity[1], o.projectile_velocity[2]);
	}
	offset += len(result);
	if offset >= len(buffer) - 1
	{
		return "", .Invalid;
	}
	return string(buffer[:offset]), .Ok;
}

split_once :: proc(value: string, separator: byte) -> (string, string, Admission)
{
	for c, index in transmute([]byte)value
	{
		if c == separator
		{
			return value[:index], value[index+1:], .Ok;
		}
	}
	return value, "", .Invalid;
}

finite_number :: proc(value: f32) -> Admission
{
 if transmute(u32)value & 0x7f80_0000 == 0x7f80_0000
 {
  return .Invalid;
 }
 return .Ok;
}

inertia_admission :: proc(kind: Shape, size: [3]f32, density: f32) -> Admission
{
 // the cuboid hull has the box's analytic mass distribution. cooking remains
 // setup work. this allocation-free facade check validates numeric mass/inertia
 mass := density * shape_volume(kind, size);
 inertia: entasis.Body_Inertia;
 status: entasis.Status;
 switch kind
 {
 case .Box, .Hull: inertia, status = entasis.shape_inertia(entasis.box(size[0], size[1], size[2]), mass);
 case .Sphere: inertia, status = entasis.shape_inertia(entasis.sphere(size[0]/2), mass);
 case .Capsule: inertia, status = entasis.shape_inertia(entasis.capsule(size[0]/2, size[1]-size[0]), mass);
 case .Cylinder: inertia, status = entasis.shape_inertia(entasis.cylinder(size[0]/2, size[1]), mass);
 }
 if status != .Ok
 {
  return .Invalid;
 }
 values := [4]f32{inertia.inverse_mass, inertia.inverse_inertia_tensor.xx, inertia.inverse_inertia_tensor.yy, inertia.inverse_inertia_tensor.zz};
 for value in values
 {
  if value <= 0 || finite_number(value) != .Ok
  {
   return .Invalid;
  }
 }
 return .Ok;
}

RECORDING_ENABLED :: #config(ENTASIS_BENCHMARK_RECORDING, false);
SOURCE_REVISION :: #config(ENTASIS_BENCHMARK_SOURCE_REVISION, "Unknown");
BUILD_CONFIGURATION :: #config(ENTASIS_BENCHMARK_CONFIGURATION, "Unknown");
Recording_Mode :: enum
{
	Off, On,
}
Recording_Compression :: enum
{
	LZ4, LZ4HC,
}
Recording_Options :: struct
{
	mode: Recording_Mode,
	output: string,
	compression: Recording_Compression,
	budget: u64,
}

recording_supported :: proc(package_name: string) -> Admission
{
	switch package_name
	{
	case "container", "contact_islands", "pyramid", "ragdoll_stair_tumble", "noncontact_constraint_mix", "noncontact_fallback_smoke":
		return .Ok;
	}
	return .Invalid;
}

recording_arguments :: proc(package_name: string, arguments: []string,
	compiled: Recording_Mode = Recording_Mode.On when RECORDING_ENABLED else Recording_Mode.Off) -> (Recording_Options, []string, Admission)
{
	options: Recording_Options = {budget=512*1024*1024};
	remaining: []string = make([]string, len(arguments), context.temp_allocator);
	count: int;
	seen: u32;
	names: [4]string = {"--record", "--recording-output", "--compression", "--memory-mib"};
	for argument in arguments
	{
		key, value, split := split_once(argument, '=');
		index: int = -1;
		for name, candidate in names
		{
			if key == name
			{
				index = candidate;
				break;
			}
		}
		if index < 0
		{
			remaining[count] = argument;
			count += 1;
			continue;
		}
		if split != .Ok || len(value) == 0 || seen & (1<<u32(index)) != 0
		{
			return {}, nil, .Invalid;
		}
		seen |= 1<<u32(index);
		switch index
		{
		case 0:
			if value != "off" && value != "on"
			{
				return {}, nil, .Invalid;
			}
			options.mode = .On if value == "on" else .Off;
		case 1: options.output = value;
		case 2:
			if value != "lz4" && value != "lz4hc"
			{
				return {}, nil, .Invalid;
			}
			options.compression = .LZ4HC if value == "lz4hc" else .LZ4;
		case 3:
			amount: int;
			status: Admission;
			amount, status = parse_integer(value);
			if status != .Ok || amount < 64 || amount > 16384
			{
				return {}, nil, .Invalid;
			}
			options.budget = u64(amount)*1024*1024;
		}
	}
	if options.mode != compiled || (options.mode == .Off && seen & 14 != 0) ||
		(options.mode == .On && (recording_supported(package_name) != .Ok || len(options.output) == 0))
	{
		return {}, nil, .Invalid;
	}
	if options.mode == .On
	{
		name: string = os.base(options.output);
		if name == "." || name == ".." || strings.has_suffix(name, ".partial")
		{
			return {}, nil, .Invalid;
		}
		for c in name
		{
			if !(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9') && c != '_' && c != '-' && c != '.'
			{
				return {}, nil, .Invalid;
			}
		}
	}
	return options, remaining[:count], .Ok;
}

parse_options :: proc(workload: Workload, arguments: []string) -> (Options, string)
{
	names: [Workload]string = {.Container="container", .Contact_Islands="contact_islands", .Pyramid="pyramid"};
	recording: Recording_Options;
	remaining: []string;
	status: Admission;
	recording, remaining, status = recording_arguments(names[workload], arguments);
	if status != .Ok
	{
		return {}, "recording options do not match producer capability";
	}
	options: Options;
	diagnostic: string;
	options, diagnostic = parse_workload_options(workload, remaining);
	options.recording = recording;
	if recording.mode == .On && os.dir(recording.output) != os.dir(options.output)
	{
		return {}, "recording and CSV must share a run directory";
	}
	return options, diagnostic;
}

options_from_saved_parameters :: proc(workload: Workload, parameters: string, workers: int) -> (Options, Admission)
{
	if len(parameters) == 0 || len(parameters) >= PARAMETER_CAPACITY || parameters[len(parameters)-1] == ';'
	{
		return {}, .Invalid;
	}
	// each accepted option adds only its two argv prefix bytes to saved text
	storage: [PARAMETER_CAPACITY+2*len(OPTION_NAMES)]u8;
	arguments: [len(OPTION_NAMES)]string;
	argument_count, offset, field_count: int;
	remaining: string = parameters;
	for field in strings.split_by_byte_iterator(&remaining, ';')
	{
		key, value, status := split_once(field, '=');
		if status != .Ok || len(key) == 0 || len(value) == 0
		{
			return {}, .Invalid;
		}
		previous_fields: string = parameters;
		for previous_index: int = 0; previous_index < field_count; previous_index += 1
		{
			previous, _ := strings.split_by_byte_iterator(&previous_fields, ';');
			previous_key, _, _ := split_once(previous, '=');
			if key == previous_key
			{
				return {}, .Invalid;
			}
		}
		field_count += 1;
		for option in OPTION_NAMES
		{
			if key == option && key != "worker-count" && key != "output"
			{
				if argument_count >= len(arguments)-2 || len(field)+2 > len(storage)-offset
				{
					return {}, .Invalid;
				}
				storage[offset], storage[offset+1] = '-', '-';
				copy(storage[offset+2:], field);
				arguments[argument_count] = string(storage[offset:offset+2+len(field)]);
				argument_count += 1;
				offset += 2+len(field);
			}
		}
	}
	worker_buffer: [len("--worker-count=")+20]u8;
	arguments[argument_count] = fmt.bprintf(worker_buffer[:], "--worker-count=%d", workers);
	arguments[argument_count+1] = "--output=preview.csv";
	options, diagnostic := parse_workload_options(workload, arguments[:argument_count+2]);
	if len(diagnostic) != 0
	{
		return {}, .Invalid;
	}
	remaining = parameters;
	for field in strings.split_by_byte_iterator(&remaining, ';')
	{
		if field == "timing=native-step-sum" && workload == .Container
		{
			options.recording.mode = .On;
		}
	}
	buffer: [PARAMETER_CAPACITY]u8;
	canonical, status := canonical_parameters(options, buffer[:]);
	if status != .Ok
	{
		return {}, .Invalid;
	}
	expected_count: int;
	for field in strings.split_by_byte_iterator(&canonical, ';')
	{
		expected_count += 1;
		found: Admission = .Invalid;
		remaining = parameters;
		for supplied in strings.split_by_byte_iterator(&remaining, ';')
		{
			if supplied == field
			{
				found = .Ok;
				break;
			}
		}
		if found != .Ok
		{
			return {}, .Invalid;
		}
	}
	if expected_count != field_count
	{
		return {}, .Invalid;
	}
	return options, .Ok;
}
