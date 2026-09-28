package main

import "core:encoding/json"
import "core:fmt"
import "core:os"
import "core:path/filepath"
import "core:strconv"
import "core:strings"

External_Flag :: enum
{
	Unset,
	Disabled,
	Enabled,
}

Node_Kind :: enum
{
	Invalid,
	Null,
	Integer,
	String,
	Array,
	Object,
	External_Flag,
}

Node_Entry :: struct
{
	key:   string,
	value: ^Node,
}

Node :: struct
{
	kind:         Node_Kind,
	integer:      i64,
	text:         string,
	external_flag: External_Flag,
	items:        [dynamic]^Node,
	entries:      [dynamic]Node_Entry,
}

Manifest :: struct
{
	root:      ^Node,
	root_path: string,
}

Parse_Result :: enum
{
	Ok,
	Invalid,
	Read_Failed,
}

@(private)
node_new :: proc(kind: Node_Kind) -> ^Node
{
	node := new(Node)
	node.kind = kind
	return node
}

@(private)
parse_node :: proc(parser: ^json.Parser) -> (^Node, json.Error)
{
	token := parser.curr_token
	#partial switch token.kind
	{
		case .Null:
		_, err := json.advance_token(parser)
		return node_new(.Null), err
		case .False:
		_, err := json.advance_token(parser)
		node := node_new(.External_Flag)
		node.external_flag = .Disabled
		return node, err
		case .True:
		_, err := json.advance_token(parser)
		node := node_new(.External_Flag)
		node.external_flag = .Enabled
		return node, err
		case .Integer:
		_, err := json.advance_token(parser)
		if err != nil && err != .EOF
		{
			return nil, err
		}
		value, _ := strconv.parse_i64(token.text)
		node := node_new(.Integer)
		node.integer = value
		node.text = token.text
		return node, nil
		case .String:
		value, err := json.unquote_string(token, .JSON)
		if err != nil
		{
			return nil, err
		}
		_, advance_error := json.advance_token(parser)
		if advance_error != nil && advance_error != .EOF
		{
			return nil, advance_error
		}
		node := node_new(.String)
		node.text = value
		return node, nil
		case .Open_Bracket:
		_, err := json.advance_token(parser)
		if err != nil && err != .EOF
		{
			return nil, err
		}
		node := node_new(.Array)
		for parser.curr_token.kind != .Close_Bracket
		{
			item, item_error := parse_node(parser)
			if item_error != nil
			{
				return nil, item_error
			}
			append(&node.items, item)
			if parser.curr_token.kind == .Comma
			{
				_, comma_error := json.advance_token(parser)
				if comma_error != nil && comma_error != .EOF
				{
					return nil, comma_error
				}
				if parser.curr_token.kind == .Close_Bracket
				{
					return nil, .Unexpected_Token
				}
			}
			else if parser.curr_token.kind != .Close_Bracket
			{
				return nil, .Unexpected_Token
			}
		}
		_, close_error := json.advance_token(parser)
		if close_error != nil && close_error != .EOF
		{
			return nil, close_error
		}
		return node, nil
		case .Open_Brace:
		_, err := json.advance_token(parser)
		if err != nil && err != .EOF
		{
			return nil, err
		}
		node := node_new(.Object)
		for parser.curr_token.kind != .Close_Brace
		{
			if parser.curr_token.kind != .String
			{
				return nil, .Expected_String_For_Object_Key
			}
			key, key_error := json.unquote_string(parser.curr_token, .JSON)
			if key_error != nil
			{
				return nil, key_error
			}
			for entry in node.entries
			{
				if entry.key == key
				{
					return nil, .Duplicate_Object_Key
				}
			}
			_, key_advance_error := json.advance_token(parser)
			if key_advance_error != nil || parser.curr_token.kind != .Colon
			{
				return nil, .Expected_Colon_After_Key
			}
			_, colon_error := json.advance_token(parser)
			if colon_error != nil && colon_error != .EOF
			{
				return nil, colon_error
			}
			value, value_error := parse_node(parser)
			if value_error != nil
			{
				return nil, value_error
			}
			append(&node.entries, Node_Entry{key = key, value = value})
			if parser.curr_token.kind == .Comma
			{
				_, comma_error := json.advance_token(parser)
				if comma_error != nil && comma_error != .EOF
				{
					return nil, comma_error
				}
				if parser.curr_token.kind == .Close_Brace
				{
					return nil, .Unexpected_Token
				}
			}
			else if parser.curr_token.kind != .Close_Brace
			{
				return nil, .Unexpected_Token
			}
		}
		_, close_error := json.advance_token(parser)
		if close_error != nil && close_error != .EOF
		{
			return nil, close_error
		}
		return node, nil
		case:
		return nil, .Unexpected_Token
	}
}

manifest_load :: proc(path: string) -> (Manifest, Parse_Result)
{
	data, read_error := os.read_entire_file(path, context.allocator)
	if read_error != nil
	{
		fmt.eprintfln("failed to read ABI manifest: %s", path)
		return {}, .Read_Failed
	}
	parser := json.make_parser(data, .JSON, true, context.allocator) // odin-contracts-allow: external-boundary rule=ODIN_BOOLEAN_LITERAL_CONTRACT boundary=core_json_parse_integers conversion=Node_Integer
	root, parse_error := parse_node(&parser)
	if parse_error != nil || root == nil || parser.curr_token.kind != .EOF
	{
		fmt.eprintfln("invalid ABI manifest JSON: %v", parse_error)
		return {}, .Invalid
	}
	return Manifest{root = root, root_path = path}, .Ok
}

@(private)
node_field :: proc(node: ^Node, key: string) -> ^Node
{
	if node == nil || node.kind != .Object
	{
		return nil
	}
	for entry in node.entries
	{
		if entry.key == key
		{
			return entry.value
		}
	}
	return nil
}

node_string :: proc(node: ^Node) -> string
{
	if node == nil || node.kind != .String
	{
		return ""
	}
	return node.text
}

node_integer :: proc(node: ^Node) -> i64
{
	if node == nil || node.kind != .Integer
	{
		return 0
	}
	return node.integer
}

node_items :: proc(node: ^Node) -> []^Node
{
	if node == nil || node.kind != .Array
	{
		return nil
	}
	return node.items[:]
}

node_entries :: proc(node: ^Node) -> []Node_Entry
{
	if node == nil || node.kind != .Object
	{
		return nil
	}
	return node.entries[:]
}

field_string :: proc(node: ^Node, key: string) -> string
{
	return node_string(node_field(node, key))
}

field_integer :: proc(node: ^Node, key: string) -> i64
{
	return node_integer(node_field(node, key))
}

field_items :: proc(node: ^Node, key: string) -> []^Node
{
	return node_items(node_field(node, key))
}

field_entries :: proc(node: ^Node, key: string) -> []Node_Entry
{
	return node_entries(node_field(node, key))
}

field_flag :: proc(node: ^Node, key: string) -> External_Flag
{
	value := node_field(node, key)
	if value == nil || value.kind != .External_Flag
	{
		return .Unset
	}
	return value.external_flag
}

@(private)
root_field :: proc(manifest: ^Manifest, key: string) -> ^Node
{
	return node_field(manifest.root, key)
}

@(private)
header_spec :: proc(manifest: ^Manifest, header: string) -> ^Node
{
	return node_field(root_field(manifest, "header_layout"), header)
}

@(private)
library_spec :: proc(manifest: ^Manifest, library: string) -> ^Node
{
	return node_field(root_field(manifest, "libraries"), library)
}

contains_string :: proc(values: []string, target: string) -> External_Flag
{
	for value in values
	{
		if value == target
		{
			return .Enabled
		}
	}
	return .Disabled
}

node_array_contains_string :: proc(node: ^Node, target: string) -> External_Flag
{
	for item in node_items(node)
	{
		if node_string(item) == target
		{
			return .Enabled
		}
	}
	return .Disabled
}

header_dependencies :: proc(manifest: ^Manifest, header: string) -> [dynamic]string
{
	result: [dynamic]string
	pending: [dynamic]string
	for dependency in field_items(header_spec(manifest, header), "includes")
	{
		append(&pending, node_string(dependency))
	}
	for len(pending) > 0
	{
		dependency := pop(&pending)
		if contains_string(result[:], dependency) == .Enabled
		{
			continue
		}
		append(&result, dependency)
		for nested in field_items(header_spec(manifest, dependency), "includes")
		{
			append(&pending, node_string(nested))
		}
	}
	return result
}

header_has :: proc(manifest: ^Manifest, owner, dependency: string) -> External_Flag
{
	if owner == dependency
	{
		return .Enabled
	}
	dependencies := header_dependencies(manifest, owner)
	return contains_string(dependencies[:], dependency)
}

c_named_type :: proc(c_type: string) -> (string, External_Flag)
{
	pointer: External_Flag = .Disabled
	if strings.contains(c_type, "*")
	{
		pointer = .Enabled
	}
	without_const, _ := strings.replace_all(c_type, "const", " ")
	without_volatile, _ := strings.replace_all(without_const, "volatile", " ")
	without_pointer, _ := strings.replace_all(without_volatile, "*", " ")
	parts := strings.fields(without_pointer)
	return strings.join(parts, " "), pointer
}

@(private)
find_named_item :: proc(items: []^Node, name: string) -> ^Node
{
	for item in items
	{
		if field_string(item, "name") == name
		{
			return item
		}
	}
	return nil
}

documentation_normalize :: proc(text: string, symbol: External_Flag = .Disabled) -> string
{
	start := 0
	end := len(text)
	if symbol == .Enabled && strings.has_prefix(text, "entasis_")
	{
		start = len("entasis_")
	}
	if symbol == .Enabled && end - start > 2 && strings.has_suffix(text[:end], "_t")
	{
		end -= 2
	}
	builder := strings.builder_make_len_cap(0, end - start)
	space_pending := External_Flag.Disabled
	written := External_Flag.Disabled
	for character in text[start:end]
	{
		value := character
		if value >= 'A' && value <= 'Z'
		{
			value += 'a' - 'A'
		}
		if (value >= 'a' && value <= 'z') || (value >= '0' && value <= '9')
		{
			if space_pending == .Enabled && written == .Enabled
			{
				strings.write_byte(&builder, ' ')
			}
			strings.write_byte(&builder, u8(value))
			space_pending = .Disabled
			written = .Enabled
		}
		else
		{
			space_pending = .Enabled
		}
	}
	return strings.to_string(builder)
}

documentation_validate :: proc(item: ^Node, identity, label: string) -> (External_Flag, string)
{
	doc := field_string(item, "doc")
	if doc == ""
	{
		return .Disabled, fmt.aprintf("missing documentation for %s %s", label, identity)
	}
	if documentation_normalize(doc) == documentation_normalize(identity, .Enabled)
	{
		return .Disabled, fmt.aprintf("placeholder documentation for %s %s", label, identity)
	}
	return .Enabled, ""
}

validate_manifest :: proc(manifest: ^Manifest) -> (External_Flag, string)
{
	root := manifest.root
	if root == nil || root.kind != .Object
	{
		return .Disabled, "unsupported ABI manifest schema"
	}
	if field_integer(root, "schema_version") != 1
	{
		return .Disabled, "unsupported ABI manifest schema"
	}
	product := node_field(root, "product")
	integer_fields := []string{"version_major", "version_minor", "version_patch", "abi_version"}
	for name in integer_fields
	{
		value := node_field(product, name)
		minimum: i64 = 0
		maximum: i64 = 65535
		if name == "abi_version"
		{
			minimum = 1
			maximum = 4294967295
		}
		if value == nil || value.kind != .Integer || value.integer < minimum || value.integer > maximum ||
		value.text != fmt.aprintf("%d", value.integer)
		{
			return .Disabled, fmt.aprintf("product.%s must be an integer in %d..%d", name, minimum, maximum)
		}
	}
	string_fields := []string{"version_prerelease", "version_string"}
	for name in string_fields
	{
		value := node_field(product, name)
		if value == nil || value.kind != .String
		{
			return .Disabled, fmt.aprintf("product.%s must be a string", name)
		}
	}
	prerelease := field_string(product, "version_prerelease")
	if prerelease != ""
	{
		for identifier in strings.split(prerelease, ".")
		{
			if identifier == ""
			{
				return .Disabled, "product.version_prerelease contains an empty identifier"
			}
			numeric := External_Flag.Enabled
			for character in identifier
			{
				if character >= '0' && character <= '9'
				{
					continue
				}
				if character != '-' && !(character >= 'A' && character <= 'Z') && !(character >= 'a' && character <= 'z')
				{
					return .Disabled, "product.version_prerelease contains an invalid character"
				}
				numeric = .Disabled
			}
			if numeric == .Enabled && len(identifier) > 1 && identifier[0] == '0'
			{
				return .Disabled, "product.version_prerelease numeric identifiers must not have leading zeroes"
			}
		}
	}
	expected_version := fmt.aprintf("%d.%d.%d", field_integer(product, "version_major"), field_integer(product, "version_minor"), field_integer(product, "version_patch"))
	if prerelease != ""
	{
		expected_version = fmt.aprintf("%s-%s", expected_version, prerelease)
	}
	if field_string(product, "version_string") != expected_version
	{
		return .Disabled, fmt.aprintf("product.version_string must equal %s", expected_version)
	}
	for library in field_entries(root, "libraries")
	{
		prefix := "ENTASIS"
		if library.key == "cooking"
		{
			prefix = "ENTASIS_COOKING"
		}
		expected_node := fmt.aprintf("%s_%d", prefix, field_integer(product, "abi_version"))
		if field_string(library.value, "version_node") != expected_node
		{
			return .Disabled, fmt.aprintf("libraries.%s.version_node must equal %s", library.key, expected_node)
		}
	}
	identity_sections := []string{"structs", "functions", "callback_types"}
	for section_name in identity_sections
	{
		items := field_items(root, section_name)
		for item, index in items
		{
			name := field_string(item, "name")
			for previous in items[:index]
			{
				if field_string(previous, "name") == name
				{
					label := section_name
					if label == "structs"
					{
						label = "struct"
					}
					if label == "functions"
					{
						label = "function"
					}
					if label == "callback_types"
					{
						label = "callback"
					}
					return .Disabled, fmt.aprintf("duplicate %s names: ['%s']", label, name)
				}
			}
		}
	}
	functions := field_items(root, "functions")
	for function in functions
	{
		name := field_string(function, "name")
		library := field_string(function, "library")
		header := field_string(function, "header")
		if !strings.has_prefix(name, "entasis_")
		{
			return .Disabled, fmt.aprintf("invalid export prefix: %s", name)
		}
		if library != "runtime" && library != "cooking"
		{
			return .Disabled, fmt.aprintf("unsupported library: %s", library)
		}
		surface := field_string(function, "surface")
		if surface == ""
		{
			surface = library
		}
		if surface != "runtime" && surface != "cooking"
		{
			return .Disabled, fmt.aprintf("unsupported surface: %s", surface)
		}
		if library == "runtime" && header == "cooking"
		{
			return .Disabled, fmt.aprintf("runtime function assigned to cooking header: %s", name)
		}
		if library == "cooking" && header != "cooking"
		{
			return .Disabled, fmt.aprintf("cooking function assigned outside cooking header: %s", name)
		}
	}
	headers := root_field(manifest, "header_layout")
	if headers == nil || headers.kind != .Object || len(headers.entries) == 0
	{
		return .Disabled, "missing header_layout"
	}
	required_headers := []string{"base", "collision", "world", "shapes", "bodies", "constraints", "queries", "events", "views", "properties", "cooking", "entasis"}
	if len(headers.entries) != len(required_headers)
	{
		return .Disabled, "unexpected public header set"
	}
	for required in required_headers
	{
		if node_field(headers, required) == nil
		{
			return .Disabled, "unexpected public header set"
		}
	}
	for entry in headers.entries
	{
		name := entry.key
		spec := entry.value
		path := field_string(spec, "path")
		if !strings.has_prefix(path, "include/entasis/")
		{
			return .Disabled, fmt.aprintf("invalid public header path for %s: %s", name, path)
		}
		if field_string(spec, "guard") == ""
		{
			return .Disabled, fmt.aprintf("missing include guard for %s", name)
		}
		for dependency_node in field_items(spec, "includes")
		{
			dependency := node_string(dependency_node)
			if node_field(headers, dependency) == nil
			{
				return .Disabled, fmt.aprintf("unknown header dependency %s -> %s", name, dependency)
			}
		}
		dependencies := header_dependencies(manifest, name)
		if contains_string(dependencies[:], name) == .Enabled
		{
			return .Disabled, fmt.aprintf("public header dependency cycle at %s", name)
		}
	}
	declaration_sections := []string{"scalar_aliases", "enum_constants", "macro_constants", "callback_types", "structs", "struct_aliases", "functions"}
	for section_name in declaration_sections
	{
		for item in field_items(root, section_name)
		{
			identity := field_string(item, "name")
			if identity == ""
			{
				identity = field_string(item, "type")
			}
			if documented, documentation_error := documentation_validate(item, identity, section_name);
			documented == .Disabled
			{
				return .Disabled, documentation_error
			}
			header := field_string(item, "header")
			if node_field(headers, header) == nil || header == "entasis"
			{
				if identity == ""
				{
					identity = "<unnamed>"
				}
				return .Disabled, fmt.aprintf("invalid header assignment for %s %s: %s", section_name, identity, header)
			}
		}
	}
	for enum_group in field_items(root, "enum_constants")
	{
		value_docs := node_field(enum_group, "value_docs")
		if value_docs == nil || value_docs.kind != .Object
		{
			return .Disabled, fmt.aprintf("missing enum value documentation for %s", field_string(enum_group, "type"))
		}
		for value in field_items(enum_group, "values")
		{
			pair := node_items(value)
			name := node_string(pair[0])
			doc := node_string(node_field(value_docs, name))
			if doc == "" || documentation_normalize(doc) == documentation_normalize(name, .Enabled)
			{
				return .Disabled, fmt.aprintf("missing or placeholder enum value documentation for %s", name)
			}
		}
	}
	for callback in field_items(root, "callback_types")
	{
		for parameter in field_items(callback, "params")
		{
			if field_string(parameter, "doc") == ""
			{
				return .Disabled, fmt.aprintf("missing parameter documentation for %s.%s", field_string(callback, "name"), field_string(parameter, "name"))
			}
		}
		if field_string(callback, "return_doc") == ""
		{
			return .Disabled, fmt.aprintf("missing return documentation for %s", field_string(callback, "name"))
		}
	}
	for item in field_items(root, "structs")
	{
		alignment := node_field(item, "explicit_alignment")
		if alignment != nil && (alignment.kind != .Integer || alignment.integer <= 0 ||
			(alignment.integer & (alignment.integer - 1)) != 0 ||
			alignment.integer != field_integer(item, "align") || len(field_items(item, "fields")) == 0)
		{
			return .Disabled, fmt.aprintf("invalid explicit alignment for %s", field_string(item, "name"))
		}
		for field in field_items(item, "fields")
		{
			if field_string(field, "doc") == ""
			{
				return .Disabled, fmt.aprintf("missing field documentation for %s.%s", field_string(item, "name"), field_string(field, "name"))
			}
		}
	}
	for function in field_items(root, "functions")
	{
		for parameter in field_items(function, "params")
		{
			if field_string(parameter, "doc") == ""
			{
				return .Disabled, fmt.aprintf("missing parameter documentation for %s.%s", field_string(function, "name"), field_string(parameter, "name"))
			}
		}
		if field_string(function, "return_doc") == ""
		{
			return .Disabled, fmt.aprintf("missing return documentation for %s", field_string(function, "name"))
		}
	}
	structs_nodes := field_items(root, "structs")
	callbacks := field_items(root, "callback_types")
	for item in structs_nodes
	{
		owner := field_string(item, "header")
		for field in field_items(item, "fields")
		{
			named, pointer := c_named_type(field_string(field, "c_type"))
			struct_dependency := find_named_item(structs_nodes, named)
			if struct_dependency != nil && pointer == .Disabled && header_has(manifest, owner, field_string(struct_dependency, "header")) == .Disabled
			{
				return .Disabled, fmt.aprintf("%s in %s needs complete %s from %s", field_string(item, "name"), owner, named, field_string(struct_dependency, "header"))
			}
			callback_dependency := find_named_item(callbacks, named)
			if callback_dependency != nil && header_has(manifest, owner, field_string(callback_dependency, "header")) == .Disabled
			{
				return .Disabled, fmt.aprintf("%s in %s needs callback %s from %s", field_string(item, "name"), owner, named, field_string(callback_dependency, "header"))
			}
		}
	}
	for callback in callbacks
	{
		owner := field_string(callback, "header")
		entries: [dynamic]^Node
		append(&entries, ..field_items(callback, "params"))
		if field_string(callback, "return_c") != "void"
		{
			return_node := node_new(.Object)
			value_node := node_new(.String)
			value_node.text = field_string(callback, "return_c")
			append(&return_node.entries, Node_Entry{key = "c_type", value = value_node})
			append(&entries, return_node)
		}
		for entry in entries
		{
			named, pointer := c_named_type(field_string(entry, "c_type"))
			struct_dependency := find_named_item(structs_nodes, named)
			if struct_dependency != nil && pointer == .Disabled && header_has(manifest, owner, field_string(struct_dependency, "header")) == .Disabled
			{
				return .Disabled, fmt.aprintf("callback %s in %s needs complete %s from %s", field_string(callback, "name"), owner, named, field_string(struct_dependency, "header"))
			}
			callback_dependency := find_named_item(callbacks, named)
			if callback_dependency != nil && header_has(manifest, owner, field_string(callback_dependency, "header")) == .Disabled
			{
				return .Disabled, fmt.aprintf("callback %s in %s needs %s from %s", field_string(callback, "name"), owner, named, field_string(callback_dependency, "header"))
			}
		}
	}
	for function in functions
	{
		owner := field_string(function, "header")
		entries: [dynamic]^Node
		append(&entries, ..field_items(function, "params"))
		if field_string(function, "return_c") != "void"
		{
			return_node := node_new(.Object)
			value_node := node_new(.String)
			value_node.text = field_string(function, "return_c")
			append(&return_node.entries, Node_Entry{key = "c_type", value = value_node})
			append(&entries, return_node)
		}
		for entry in entries
		{
			named, pointer := c_named_type(field_string(entry, "c_type"))
			struct_dependency := find_named_item(structs_nodes, named)
			if struct_dependency != nil && pointer == .Disabled && header_has(manifest, owner, field_string(struct_dependency, "header")) == .Disabled
			{
				return .Disabled, fmt.aprintf("function %s in %s needs complete %s from %s", field_string(function, "name"), owner, named, field_string(struct_dependency, "header"))
			}
			callback_dependency := find_named_item(callbacks, named)
			if callback_dependency != nil && header_has(manifest, owner, field_string(callback_dependency, "header")) == .Disabled
			{
				return .Disabled, fmt.aprintf("function %s in %s needs callback %s from %s", field_string(function, "name"), owner, named, field_string(callback_dependency, "header"))
			}
		}
	}
	return .Enabled, ""
}

manifest_path :: proc(root: string) -> string
{
	path, _ := filepath.join([]string{root, "tools", "abi", "abi_manifest.json"})
	return path
}
