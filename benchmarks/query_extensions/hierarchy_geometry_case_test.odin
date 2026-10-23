package query_extensions

import "core:testing"
import "core:fmt"
import "core:math"
import e "entasis:entasis"

@(test)
hierarchy_geometry_oracles_reject_invalid_payloads :: proc(t: ^testing.T)
{
	// bounded untimed probes exercise geometry/oracles without benchmark dispatch
	for operation in ([6]string{"closest-point", "distance", "depenetration", "sweep-any", "sweep-closest", "sweep-all"})
	{
		for cell in ([7]string{"nested-query-64", "query-box-1", "compound4-nested64", "compound4-mesh128", "compound4-nested64-angular", "compound4-mesh128-angular", "invalid"})
		{
			if hierarchy_geometry_admit(operation, cell) != .Ok
			{
				continue;
			}
			for query_case in ([2]string{"hierarchy", "global"})
			{
				if query_case == "global" && global_cell_admit(query_case, operation, cell) != .Ok
				{
					continue;
				}
				poses: [32]e.Rigid_Pose;
				observations: [32]Hierarchy_Geometry_Observation;
				contexts: [1]e.Query_Context;
				fixture: Hierarchy_Geometry_Case = {args={query_case=query_case, query_operation=operation, query_cell=cell, workers=1}, contexts=contexts[:], poses=poses[:], observations=observations[:]};
				if !testing.expect_value(t, hierarchy_geometry_prepare(&fixture), e.Status.Ok)
				{
					return;
				}
				for index in 0..<len(poses)
				{
					fixture.observations[index] = hierarchy_geometry_query(&fixture, 0, index);
					validation: e.Status = hierarchy_geometry_validate(&fixture, index);
					if validation != .Ok
					{
						fmt.eprintfln("geometry oracle case=%s operation=%s cell=%s index=%d result=%v", query_case, operation, cell, index, fixture.observations[index]);
					}
					testing.expect_value(t, validation, e.Status.Ok);
				}
				good: Hierarchy_Geometry_Observation = fixture.observations[0];
				fixture.observations[0].status = .Capacity_Missing;
				testing.expect_value(t, hierarchy_geometry_validate(&fixture, 0), e.Status.Invalid_Description);
				fixture.observations[0] = good;
				fixture.observations[0].count = 0;
				testing.expect_value(t, hierarchy_geometry_validate(&fixture, 0), e.Status.Invalid_Description);
				fixture.observations[0] = good;
				if operation == "depenetration"
				{
					fixture.observations[0].correction.translation.y = math.nan_f32();
				}
				else if operation == "sweep-closest" || operation == "sweep-all"
				{
					fixture.observations[0].sweep.sweep.t1 = math.nan_f32();
				}
				else if operation != "sweep-any"
				{
					fixture.observations[0].shape.geometry.distance = math.nan_f32();
				}
				if operation != "sweep-any"
				{
					testing.expect_value(t, hierarchy_geometry_validate(&fixture, 0), e.Status.Invalid_Description);
				}
				if operation == "sweep-closest" || operation == "sweep-all"
				{
					fixture.observations[0] = good;
					for normal_y in ([2]f32{math.inf_f32(1), 2})
					{
						fixture.observations[0].sweep.sweep.normal.y = normal_y;
						testing.expect_value(t, hierarchy_geometry_validate(&fixture, 0), e.Status.Invalid_Description);
					}
					for normal_y in ([3]f32{0.999, 1, 1.001})
					{
						fixture.observations[0].sweep.sweep.normal.y = normal_y;
						testing.expect_value(t, hierarchy_geometry_validate(&fixture, 0), e.Status.Ok);
					}
				}
				e.query_context_destroy(&fixture.contexts[0]);
				e.world_destroy(&fixture.world);
			}
		}
	}
}
