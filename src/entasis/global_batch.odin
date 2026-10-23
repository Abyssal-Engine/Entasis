#+vet !unused-imports

package entasis

import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// positions contains one simulation-frame f64 value per description and
	// overrides its local position. handles has at least len(descriptions) entries
	// invalid stream lengths leave handles untouched. after length admission,
	// uncommitted slots in handles[:len(descriptions)] are invalid
	// rows commit in input order and return the completed count on success or failure
	// handles[len(descriptions):] is unchanged. capacity growth may allocate
	// requires exclusive idle-world ownership
	body_add_batch_global :: proc (
		world: ^World,
		descriptions: []Body_Description,
		handles: []Body_Handle,
		positions: []Global_Position,
	) -> (int, Status)
	{
		return body_add_batch_internal(world, descriptions, handles, positions, .Global);
	}
	
	// handles, descriptions and simulation-frame positions have matching lengths
	// high positions override local positions. a failed row stops the batch and
	// returns the committed prefix, earlier mutations remain applied
	// mutation may allocate. requires exclusive idle-world ownership
	body_apply_batch_global :: proc (
		world: ^World,
		handles: []Body_Handle,
		descriptions: []Body_Description,
		positions: []Global_Position,
	) -> (int, Status)
	{
		return body_apply_batch_internal(world, handles, descriptions, positions, .Global);
	}
	
	// positions contains one simulation-frame f64 value per description and
	// overrides its local position. handles has at least len(descriptions) entries
	// invalid lengths or awakening policy leave handles untouched. after these
	// checks, uncommitted slots in handles[:len(descriptions)] are invalid
	// rows commit in input order with the selected policy and return the completed
	// count on success or failure. handles[len(descriptions):] is unchanged
	// capacity growth may allocate. requires exclusive idle-world ownership
	static_add_batch_global :: proc (
		world: ^World,
		descriptions: []Static_Description,
		handles: []Static_Handle,
		positions: []Global_Position,
		awakening: Awakening_Policy = .Overlaps,
	) -> (int, Status)
	{
		return static_add_batch_internal(world, descriptions, handles, awakening, positions, .Global);
	}
	
	// handles, descriptions and simulation-frame positions have matching lengths
	// high positions override local positions. rows use the selected awakening
	// policy, and failure returns the committed prefix without undoing earlier rows
	// requires exclusive idle-world ownership
	static_apply_batch_global :: proc (
		world: ^World,
		handles: []Static_Handle,
		descriptions: []Static_Description,
		positions: []Global_Position,
		awakening: Awakening_Policy = .Overlaps,
	) -> (int, Status)
	{
		return static_apply_batch_internal(world, handles, descriptions, awakening, positions, .Global);
	}
}
