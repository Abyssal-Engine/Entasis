package entasis_physics

import "base:intrinsics"
import util "entasis:entasis_utilities"

tree_visit_ray_leaf :: #force_inline proc "contextless" (
	leaf: $Leaf_Proc, user_context: rawptr, index: int, maximum_t: ^Tree_Scalar,
) -> Physics_Status
{
	when util.PRECISION_MODE == .Float_Coordinates
	{
		return leaf(user_context, index, maximum_t);
	}
	else when intrinsics.type_proc_parameter_type(Leaf_Proc, 2) == ^f32
	{
		local_leaf: Tree_Ray_Leaf_Proc = leaf;
		local_limit: f32 = world_precision_max_to_float(maximum_t^);
		status: Physics_Status = local_leaf(user_context, index, &local_limit);
		maximum_t^ = min(maximum_t^, f64(local_limit));
		return status;
	}
	else
	{
		global_leaf: Tree_Global_Ray_Leaf_Proc = leaf;
		return global_leaf(user_context, index, maximum_t);
	}
}


when util.PRECISION_MODE == .Double_Coordinates
{
	tree_ray_traverse :: proc
	{
	    tree_ray_traverse_coordinates,
	    tree_ray_traverse_local,
	}

	tree_ray_traverse_local :: #force_inline proc "contextless" (
	    tree: ^Tree, ray: Tree_Ray, leaf: Tree_Ray_Leaf_Proc, user_context: rawptr,
	    pool: ^util.Buffer_Pool = nil,
	) -> (f32, Physics_Status)
	{
	    limit: f64;
	    status: Physics_Status;
	    limit, status = tree_ray_traverse_coordinates(tree, tree_ray_from_local(ray), leaf, user_context, pool);
	    return f32(limit), status;
	}

	tree_ray_traverse_closest :: proc
	{
	    tree_ray_traverse_closest_coordinates,
	    tree_ray_traverse_closest_local,
	}

	tree_ray_traverse_closest_local :: #force_inline proc "contextless" (
	    tree: ^Tree, ray: Tree_Ray, leaf: Tree_Ray_Leaf_Proc, user_context: rawptr,
	    pool: ^util.Buffer_Pool = nil,
	) -> (f32, Physics_Status)
	{
	    limit: f64;
	    status: Physics_Status;
	    limit, status = tree_ray_traverse_closest_coordinates(tree, tree_ray_from_local(ray), leaf, user_context, pool);
	    return f32(limit), status;
	}

	tree_sweep_traverse :: proc
	{
	    tree_sweep_traverse_coordinates,
	    tree_sweep_traverse_local,
	}

	tree_sweep_traverse_local :: #force_inline proc "contextless" (
	    tree: ^Tree, bounds: util.Bounding_Box, sweep: util.Vector3, maximum_t: f32,
	    leaf: Tree_Ray_Leaf_Proc, user_context: rawptr, pool: ^util.Buffer_Pool = nil,
	) -> (f32, Physics_Status)
	{
	    limit: f64;
	    status: Physics_Status;
	    limit, status = tree_sweep_traverse_coordinates(tree, tree_bounds_from_local(bounds),
	        tree_position_from_local(sweep), f64(maximum_t), leaf, user_context, pool);
	    return f32(limit), status;
	}

	tree_sweep_traverse_closest :: proc
	{
	    tree_sweep_traverse_closest_coordinates,
	    tree_sweep_traverse_closest_local,
	}

	tree_sweep_traverse_closest_local :: #force_inline proc "contextless" (
	    tree: ^Tree, bounds: util.Bounding_Box, sweep: util.Vector3, maximum_t: f32,
	    leaf: Tree_Ray_Leaf_Proc, user_context: rawptr, pool: ^util.Buffer_Pool = nil,
	) -> (f32, Physics_Status)
	{
	    limit: f64;
	    status: Physics_Status;
	    limit, status = tree_sweep_traverse_closest_coordinates(tree, tree_bounds_from_local(bounds),
	        tree_position_from_local(sweep), f64(maximum_t), leaf, user_context, pool);
	    return f32(limit), status;
	}
}
else
{
	tree_ray_traverse :: tree_ray_traverse_coordinates;
	tree_ray_traverse_closest :: tree_ray_traverse_closest_coordinates;
	tree_sweep_traverse :: tree_sweep_traverse_coordinates;
	tree_sweep_traverse_closest :: tree_sweep_traverse_closest_coordinates;
}
