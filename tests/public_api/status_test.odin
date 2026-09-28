package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

@(test)
status_contract_preserves_low_level_values_and_stable_text :: proc(t: ^testing.T)
{
	cases := []struct
	{
		status: entasis.Status,
		text:   string,
	}{
		{.Ok, "ok"},
		{.Invalid_Argument, "invalid argument"},
		{.Invalid_Description, "invalid description"},
		{.Not_Found, "not found"},
		{.Capacity_Missing, "capacity missing"},
		{.Shape_In_Use, "shape in use"},
		{.Disposed, "disposed"},
		{.No_Convergence, "no convergence"},
	};
	for test_case in cases
	{
		low_level: physics.Physics_Status = test_case.status;
		public_status: entasis.Status = low_level;
		testing.expect_value(t, public_status, test_case.status);
		testing.expect_value(t, entasis.status_text(public_status), test_case.text);
		testing.expect_value(t, entasis.status_ok(public_status), public_status == .Ok);
		testing.expect_value(t, entasis.status_failed(public_status), public_status != .Ok);
	}
	testing.expect_value(t, entasis.status_text(entasis.Status(255)), "unknown status");
}

@(test)
invalid_handle_helpers_match_low_level_sentinels :: proc(t: ^testing.T)
{
	body := entasis.body_handle_invalid();
	static := entasis.static_handle_invalid();
	constraint := entasis.constraint_handle_invalid();
	shape := entasis.shape_handle_invalid();
	testing.expect_value(t, body, physics.body_handle_invalid());
	testing.expect_value(t, static, physics.static_handle_invalid());
	testing.expect_value(t, constraint, physics.constraint_handle_invalid());
	testing.expect(t, !entasis.body_handle_is_valid(body));
	testing.expect(t, !entasis.static_handle_is_valid(static));
	testing.expect(t, !entasis.constraint_handle_is_valid(constraint));
	testing.expect(t, !entasis.shape_handle_is_valid(shape));
	testing.expect(t, entasis.body_handle_is_valid({0}));
	testing.expect(t, entasis.static_handle_is_valid({0}));
	testing.expect(t, entasis.constraint_handle_is_valid({0}));
	valid_shape, status := physics.typed_index_create(0, 0);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect(t, entasis.shape_handle_is_valid(valid_shape));
}

@(test)
caller_owned_diagnostic_records_cold_failure_without_strings :: proc(t: ^testing.T)
{
	diagnostic: entasis.Diagnostic;
	entasis.diagnostic_record(
		&diagnostic,
		.Invalid_Description,
		.World_Initialize,
		3,
	);
	testing.expect_value(t, diagnostic.status, entasis.Status.Invalid_Description);
	testing.expect_value(t, diagnostic.operation, entasis.Diagnostic_Operation.World_Initialize);
	testing.expect_value(t, diagnostic.detail, i32(3));
	entasis.diagnostic_clear(&diagnostic);
	testing.expect_value(t, diagnostic.status, entasis.Status.Ok);
	testing.expect_value(t, diagnostic.operation, entasis.Diagnostic_Operation.None);
	testing.expect_value(t, diagnostic.detail, i32(0));
	entasis.diagnostic_record(nil, .Disposed, .World_Initialize);
	entasis.diagnostic_clear(nil);
}
