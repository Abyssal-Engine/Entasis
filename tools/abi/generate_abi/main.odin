package main

import "core:fmt"
import "core:mem"
import "core:os"
import "core:path/filepath"

Mode :: enum
{
	Invalid,
	Check,
	Write,
}

Output :: struct
{
	relative_path: string,
	text:          string,
}

repository_root :: proc() -> (string, os.Error)
{
	source_location := #location()
	source_directory := filepath.dir(source_location.file_path)
	relative_root, join_error := filepath.join([]string{source_directory, "..", "..", ".."})
	if join_error != nil
	{
		return "", .Invalid_Argument
	}
	return filepath.abs(relative_root, context.allocator)
}

output_add :: proc(outputs: ^[dynamic]Output, relative_path, text: string)
{
	append(outputs, Output{relative_path = relative_path, text = text})
}

expected_outputs :: proc(manifest: ^Manifest) -> ([dynamic]Output, External_Flag, string)
{
	outputs: [dynamic]Output
	output_add(&outputs, "src/entasis/version.odin", render_odin_version(manifest))
	output_add(&outputs, "include/entasis.h", render_compatibility_header("ENTASIS_H", "entasis/entasis.h"))
	output_add(&outputs, "include/entasis_cooking.h", render_compatibility_header("ENTASIS_COMPAT_COOKING_H", "entasis/cooking.h"))
	output_add(&outputs, "src/entasis_c/generated_types.odin", render_odin_types(manifest, "entasis_c"))
	output_add(&outputs, "src/entasis_cooking_c/generated_types.odin", render_odin_types(manifest, "entasis_cooking_c"))
	output_add(&outputs, "src/entasis_c/generated_exports.odin", render_odin_exports(manifest, "runtime"))
	output_add(&outputs, "src/entasis_cooking_c/generated_exports.odin", render_odin_exports(manifest, "cooking"))
	output_add(&outputs, "generated/abi/entasis.map", render_version_script(manifest, "runtime"))
	output_add(&outputs, "generated/abi/entasis.def", render_def(manifest, "runtime"))
	output_add(&outputs, "generated/abi/entasis_cooking.map", render_version_script(manifest, "cooking"))
	output_add(&outputs, "generated/abi/entasis_cooking.def", render_def(manifest, "cooking"))
	output_add(&outputs, "tests/c_abi/generated_layout_asserts.h", render_layout_header(manifest))
	for header in node_entries(root_field(manifest, "header_layout"))
	{
		text := ""
		if header.key == "entasis"
		{
			text = render_umbrella_header(manifest, header.key)
		}
		else
		{
			status: External_Flag
			error_text: string
			text, status, error_text = render_domain_header(manifest, header.key)
			if status == .Disabled
			{
				return outputs, .Disabled, error_text
			}
		}
		output_add(&outputs, field_string(header.value, "path"), text)
	}
	markdown_headers := []string{
		"base",
		"world",
		"shapes",
		"bodies",
		"collision",
		"constraints",
		"queries",
		"events",
		"views",
		"properties",
		"cooking",
	}
	for header in markdown_headers
	{
		output_add(
			&outputs,
			fmt.aprintf("docs/c/reference/%s", markdown_reference_filename(header)),
			render_c_reference(manifest, header)
		)
	}
	return outputs, .Enabled, ""
}

path_join_root :: proc(root, relative_path: string) -> string
{
	result, _ := filepath.join([]string{root, relative_path})
	return result
}

write_outputs :: proc(root: string, outputs: []Output) -> External_Flag
{
	for output in outputs
	{
		path := path_join_root(root, output.relative_path)
		if directory_error := os.make_directory_all(filepath.dir(path)); directory_error != nil && directory_error != .Exist
		{
			fmt.eprintfln("failed to create generated directory: %s (%v)", filepath.dir(path), directory_error)
			return .Disabled
		}
		if os.write_entire_file(path, output.text) != nil
		{
			fmt.eprintfln("failed to write generated file: %s", output.relative_path)
			return .Disabled
		}
	}
	return .Enabled
}

check_outputs :: proc(root: string, outputs: []Output) -> External_Flag
{
	status: External_Flag = .Enabled
	for output in outputs
	{
		path := path_join_root(root, output.relative_path)
		if !os.exists(path)
		{
			fmt.eprintfln("missing: %s", output.relative_path)
			status = .Disabled
			continue
		}
		actual, read_error := os.read_entire_file(path, context.allocator)
		if read_error != nil || string(actual) != output.text
		{
			fmt.eprintfln("stale: %s", output.relative_path)
			status = .Disabled
		}
	}
	return status
}

parse_mode :: proc(args: []string) -> Mode
{
	if len(args) != 2
	{
		return .Invalid
	}
	switch args[1]
	{
	case "--check": return .Check
	case "--write": return .Write
	case: return .Invalid
	}
}

run :: proc() -> int
{
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	defer mem.dynamic_arena_destroy(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)

	mode := parse_mode(os.args)
	if mode == .Invalid
	{
		fmt.eprintln("usage: generate_abi --write|--check")
		return 2
	}
	root, root_error := repository_root()
	if root_error != nil
	{
		fmt.eprintln("failed to resolve repository root")
		return 1
	}
	manifest, parse_result := manifest_load(manifest_path(root))
	if parse_result != .Ok
	{
		return 1
	}
	valid, validation_error := validate_manifest(&manifest)
	if valid == .Disabled
	{
		fmt.eprintln(validation_error)
		return 1
	}
	outputs, render_status, render_error := expected_outputs(&manifest)
	if render_status == .Disabled
	{
		fmt.eprintln(render_error)
		return 1
	}
	if mode == .Write
	{
		if write_outputs(root, outputs[:]) == .Disabled
		{
			return 1
		}
		fmt.printfln("generated %d files", len(outputs))
		return 0
	}
	if check_outputs(root, outputs[:]) == .Disabled
	{
		return 1
	}
	fmt.printfln("generated ABI files are current: %d files", len(outputs))
	return 0
}

main :: proc()
{
	os.exit(run())
}
