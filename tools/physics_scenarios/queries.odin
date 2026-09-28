package physics_scenarios

import observation "../physics_observation"
import entasis "entasis:entasis"
import scene "../physics_scene"

@(private)
observation_ray :: proc(o: ^observation.Observation, ray: entasis.Ray, hits: []entasis.Ray_Hit, state: scene.Overlay_Kind)
{
	origin: scene.Vec3 = observation.copy_vector(ray.origin);
	append(&o.packet.frame.overlays, scene.Overlay{kind=state, entity_a=scene.NO_ENTITY, entity_b=scene.NO_ENTITY,
		a=origin, b=origin+observation.copy_vector(ray.direction)*ray.maximum_t, value=ray.maximum_t});
	for hit in hits
	{
		point: scene.Vec3 = observation.copy_vector(hit.location);
		append(&o.packet.frame.overlays, scene.Overlay{kind=.Ray_Hit,
			entity_a=observation.observation_collidable(o, hit.collidable), entity_b=scene.NO_ENTITY,
			part_a=hit.child_index, a=point, b=point+observation.copy_vector(hit.normal), value=hit.t});
	}
}
