package main

import "core:encoding/json"
import "core:fmt"
import "core:mem"
import "core:os"
import "core:path/filepath"
import "core:strings"

Export_Format :: enum
{
	Invalid,
	Elf,
	Coff,
}

Library :: enum
{
	Invalid,
	Runtime,
	Cooking,
}

Argument_State :: struct
{
	format:        Export_Format,
	tool:          string,
	library:       Library,
	library_name:  string,
	shared_object: string,
}

Library_Spec :: struct
{
	version_node: string,
}

Libraries :: struct
{
	runtime: Library_Spec,
	cooking: Library_Spec,
}

Function :: struct
{
	library: string,
	name:    string,
}

Abi_Manifest :: struct
{
	libraries: Libraries,
	functions: []Function,
}

Parse_Status :: enum
{
	Invalid,
	Valid,
}

usage :: proc()
{
	fmt.eprintln("usage: verify_exports [--format elf|coff] [--tool PATH] runtime|cooking SHARED_OBJECT")
}

parse_arguments :: proc(args: []string) -> (Argument_State, Parse_Status)
{
	state := Argument_State{format = .Elf}
	format_count := 0
	tool_count := 0
	positionals: [dynamic]string
	index := 1
	for index < len(args)
	{
		switch args[index]
		{
		case "--format":
			format_count += 1
			if format_count > 1 || index + 1 >= len(args) || strings.has_prefix(args[index + 1], "--")
			{
				return {}, .Invalid
			}
			switch args[index + 1]
			{
			case "elf": state.format = .Elf
			case "coff": state.format = .Coff
			case: return {}, .Invalid
			}
			index += 2
		case "--tool":
			tool_count += 1
			if tool_count > 1 || index + 1 >= len(args) || args[index + 1] == "" || strings.has_prefix(args[index + 1], "--")
			{
				return {}, .Invalid
			}
			state.tool = args[index + 1]
			index += 2
		case:
			if strings.has_prefix(args[index], "--")
			{
				return {}, .Invalid
			}
			append(&positionals, args[index])
			index += 1
		}
	}
	if len(positionals) != 2
	{
		return {}, .Invalid
	}
	state.library_name = positionals[0]
	switch state.library_name
	{
	case "runtime": state.library = .Runtime
	case "cooking": state.library = .Cooking
	case: return {}, .Invalid
	}
	state.shared_object = positionals[1]
	if state.shared_object == ""
	{
		return {}, .Invalid
	}
	if state.format == .Coff && state.tool == ""
	{
		return {}, .Invalid
	}
	if state.tool == ""
	{
		state.tool = "nm"
	}
	return state, .Valid
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

sort_strings :: proc(values: []string)
{
	for index := 1; index < len(values); index += 1
	{
		current := values[index]
		position := index
		for position > 0 && strings.compare(current, values[position - 1]) < 0
		{
			values[position] = values[position - 1]
			position -= 1
		}
		values[position] = current
	}
}

contains_string :: proc(values: []string, target: string) -> Parse_Status
{
	for value in values
	{
		if value == target
		{
			return .Valid
		}
	}
	return .Invalid
}

manifest_path :: proc(root: string) -> string
{
	path, _ := filepath.join([]string{root, "tools", "abi", "abi_manifest.json"})
	return path
}

read_manifest :: proc(root: string) -> (Abi_Manifest, Parse_Status)
{
	path := manifest_path(root)
	data, read_error := os.read_entire_file(path, context.allocator)
	if read_error != nil
	{
		fmt.eprintfln("failed to read ABI manifest: %s", path)
		return {}, .Invalid
	}
	manifest: Abi_Manifest
	parse_error := json.unmarshal(data, &manifest, .JSON, context.allocator)
	if parse_error != nil
	{
		fmt.eprintfln("invalid ABI manifest JSON: %v", parse_error)
		return {}, .Invalid
	}
	return manifest, .Valid
}

expected_symbols :: proc(manifest: ^Abi_Manifest, library: string) -> [dynamic]string
{
	result: [dynamic]string
	for function in manifest.functions
	{
		if function.library == library && contains_string(result[:], function.name) == .Invalid
		{
			append(&result, function.name)
		}
	}
	sort_strings(result[:])
	return result
}

actual_symbols :: proc(arguments: ^Argument_State, manifest: ^Abi_Manifest) -> ([dynamic]string, Parse_Status)
{
	command: [dynamic]string
	append(&command, arguments.tool)
	if arguments.format == .Elf
	{
		append(&command, "-D", "--defined-only", "-j", arguments.shared_object)
	}
	else
	{
		append(&command, "--coff-exports", arguments.shared_object)
	}
	description := os.Process_Desc{command = command[:]}
	state, stdout, stderr, process_error := os.process_exec(description, context.allocator)
	if process_error != nil || !state.exited || state.exit_code != 0
	{
		if len(stderr) > 0
		{
			fmt.eprint(string(stderr))
		}
		if process_error != nil
		{
			fmt.eprintfln("export inspection failed: %v", process_error)
		}
		return nil, .Invalid
	}
	result: [dynamic]string
	lines := strings.split_lines(string(stdout))
	version_node := manifest.libraries.runtime.version_node
	if arguments.library == .Cooking
	{
		version_node = manifest.libraries.cooking.version_node
	}
	for source_line in lines
	{
		line := strings.trim_space(source_line)
		symbol := ""
		if arguments.format == .Elf
		{
			at := strings.index_byte(line, '@')
			if at >= 0
			{
				line = line[:at]
			}
			symbol = strings.trim_space(line)
			if symbol == version_node
			{
				continue
			}
		}
		else if strings.has_prefix(line, "Name:")
		{
			symbol = strings.trim_space(line[len("Name:"):])
		}
		if symbol != "" && contains_string(result[:], symbol) == .Invalid
		{
			append(&result, symbol)
		}
	}
	sort_strings(result[:])
	return result, .Valid
}

run :: proc() -> int
{
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	defer mem.dynamic_arena_destroy(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)

	arguments, argument_status := parse_arguments(os.args)
	if argument_status == .Invalid
	{
		usage()
		return 2
	}
	root, root_error := repository_root()
	if root_error != nil
	{
		fmt.eprintln("failed to resolve repository root")
		return 1
	}
	manifest, manifest_status := read_manifest(root)
	if manifest_status == .Invalid
	{
		return 1
	}
	expected := expected_symbols(&manifest, arguments.library_name)
	actual, actual_status := actual_symbols(&arguments, &manifest)
	if actual_status == .Invalid
	{
		return 1
	}
	missing: [dynamic]string
	unexpected: [dynamic]string
	for symbol in expected
	{
		if contains_string(actual[:], symbol) == .Invalid
		{
			append(&missing, symbol)
		}
	}
	for symbol in actual
	{
		if contains_string(expected[:], symbol) == .Invalid
		{
			append(&unexpected, symbol)
		}
	}
	if len(missing) > 0
	{
		fmt.printfln("missing exports: %s", strings.join(missing[:], ", "))
	}
	if len(unexpected) > 0
	{
		fmt.printfln("unexpected exports: %s", strings.join(unexpected[:], ", "))
	}
	if len(missing) > 0 || len(unexpected) > 0
	{
		return 1
	}
	fmt.printfln("%s exports: PASS (%d symbols)", arguments.library_name, len(actual))
	for symbol in actual
	{
		fmt.println(symbol)
	}
	return 0
}

main :: proc()
{
	os.exit(run())
}
