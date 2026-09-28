package main

import "core:fmt"
import "core:strings"

markdown_write_table :: proc(
	builder: ^strings.Builder,
	header: [3]string,
	rows: [][3]string,
	right_column: int = -1,
)
{
	widths: [3]int
	for cell, column in header
	{
		widths[column] = max(len(cell), 3)
		if column == right_column
		{
			widths[column] = max(widths[column], 4)
		}
	}
	for row in rows
	{
		for cell, column in row
		{
			widths[column] = max(widths[column], len(cell))
		}
	}
	for row_index in 0 ..< len(rows) + 2
	{
		row: [3]string
		if row_index == 0
		{
			row = header
		}
		else if row_index > 1
		{
			row = rows[row_index - 2]
		}
		strings.write_string(builder, "| ")
		for cell, column in row
		{
			if row_index == 1
			{
				for index in 0 ..< widths[column]
				{
					character: u8 = '-'
					if column == right_column && index == widths[column] - 1
					{
						character = ':'
					}
					strings.write_byte(builder, character)
				}
			}
			else
			{
				if column != right_column
				{
					strings.write_string(builder, cell)
				}
				for _ in 0 ..< widths[column] - len(cell)
				{
					strings.write_byte(builder, ' ')
				}
				if column == right_column
				{
					strings.write_string(builder, cell)
				}
			}
			if column < len(row) - 1
			{
				strings.write_string(builder, " | ")
			}
		}
		builder_line(builder, " |")
	}
	builder_line(builder)
}

markdown_page_title :: proc(header: string) -> string
{
	switch header
	{
	case "base":        return "Base C reference"
	case "world":       return "World C reference"
	case "shapes":      return "Shapes C reference"
	case "bodies":      return "Bodies C reference"
	case "collision":   return "Collision C reference"
	case "constraints": return "Constraints C reference"
	case "queries":     return "Queries C reference"
	case "events":      return "Events C reference"
	case "views":       return "Views C reference"
	case "properties":  return "Properties C reference"
	case "cooking":     return "Cooking C reference"
	case:                return "C reference"
	}
}

markdown_reference_filename :: proc(header: string) -> string
{
	switch header
	{
	case "base":        return "BASE.md"
	case "world":       return "WORLD.md"
	case "shapes":      return "SHAPES.md"
	case "bodies":      return "BODIES.md"
	case "collision":   return "COLLISION.md"
	case "constraints": return "CONSTRAINTS.md"
	case "queries":     return "QUERIES.md"
	case "events":      return "EVENTS.md"
	case "views":       return "VIEWS.md"
	case "properties":  return "PROPERTIES.md"
	case "cooking":     return "COOKING.md"
	case:                return ""
	}
}

markdown_c_function_declaration :: proc(function: ^Node) -> string
{
	params: [dynamic]string
	for parameter in field_items(function, "params")
	{
		append(&params, c_decl(parameter))
	}
	parameter_text := "void"
	if len(params) > 0
	{
		parameter_text = strings.join(params[:], ", ")
	}
	prefix := "ENTASIS_API"
	call := "ENTASIS_CALL"
	if field_string(function, "header") == "cooking"
	{
		prefix = "ENTASIS_COOKING_API"
		call = "ENTASIS_COOKING_CALL"
	}
	return fmt.aprintf(
		"%s %s %s %s(%s);",
		prefix,
		field_string(function, "return_c"),
		call,
		field_string(function, "name"),
		parameter_text,
	)
}

markdown_write_doc :: proc(builder: ^strings.Builder, item: ^Node)
{
	doc := field_string(item, "doc")
	if doc != ""
	{
		builder_line(builder, doc)
		builder_line(builder)
	}
	details := field_string(item, "details")
	if details != "" && details != doc
	{
		builder_line(builder, details)
		builder_line(builder)
	}
}

markdown_write_parameters :: proc(builder: ^strings.Builder, item: ^Node)
{
	params := field_items(item, "params")
	if len(params) == 0
	{
		return
	}
	rows: [dynamic][3]string
	for parameter in params
	{
		append(&rows, [3]string{
			fmt.aprintf("`%s`", field_string(parameter, "name")),
			fmt.aprintf("`%s`", field_string(parameter, "c_type")),
			field_string(parameter, "doc"),
		})
	}
	markdown_write_table(builder, {"Parameter", "C type", "Meaning"}, rows[:])
}

render_c_reference :: proc(manifest: ^Manifest, header: string) -> string
{
	builder := strings.builder_make_len_cap(0, 262144)
	builder_format_line(&builder, "<!-- %s -->", BANNER_OWNER)
	builder_line(&builder)
	builder_format_line(&builder, "# %s", markdown_page_title(header))
	builder_line(&builder)
	builder_line(&builder, "[C/C++ getting started](../README.md) | [C API guide](../API.md)")
	builder_line(&builder)
	builder_format_line(&builder, "Header: `<%s>`", header_include_path(manifest, header))
	builder_line(&builder)
	builder_line(&builder, "Use the [C API guide](../API.md) for setup and examples. The declarations below describe each type and operation. Structure sizes and alignments are in bytes")
	builder_line(&builder)

	if header == "base"
	{
		product := root_field(manifest, "product")
		builder_line(&builder, "## Product constants")
		builder_line(&builder)
		rows := [][3]string{
			{"`ENTASIS_VERSION_MAJOR`", fmt.aprintf("`%d`", field_integer(product, "version_major")), "Product major version"},
			{"`ENTASIS_VERSION_MINOR`", fmt.aprintf("`%d`", field_integer(product, "version_minor")), "Product minor version"},
			{"`ENTASIS_VERSION_PATCH`", fmt.aprintf("`%d`", field_integer(product, "version_patch")), "Product patch version"},
			{"`ENTASIS_VERSION_PRERELEASE`", fmt.aprintf("`\"%s\"`", field_string(product, "version_prerelease")), "Product prerelease identifier"},
			{"`ENTASIS_VERSION_STRING`", fmt.aprintf("`\"%s\"`", field_string(product, "version_string")), "Complete product version"},
			{"`ENTASIS_ABI_VERSION`", fmt.aprintf("`%d`", field_integer(product, "abi_version")), "Stable C ABI generation"},
			{"`ENTASIS_MAXIMUM_WORKER_COUNT`", fmt.aprintf("`%d`", field_integer(product, "maximum_worker_count")), "Maximum accepted worker count"},
		}
		markdown_write_table(&builder, {"Constant", "Value", "Meaning"}, rows)
	}
	else if header == "cooking"
	{
		builder_line(&builder, "## ABI constant")
		builder_line(&builder)
		builder_format_line(&builder, "`ENTASIS_COOKING_ABI_VERSION` is `%d`", field_integer(root_field(manifest, "product"), "abi_version"))
		builder_line(&builder)
	}

	aliases: [dynamic]^Node
	for item in field_items(manifest.root, "scalar_aliases")
	{
		if field_string(item, "header") == header
		{
			append(&aliases, item)
		}
	}
	if len(aliases) > 0
	{
		builder_line(&builder, "## Scalar aliases")
		builder_line(&builder)
		for item in aliases
		{
			builder_format_line(&builder, "### `%s`", field_string(item, "name"))
			builder_line(&builder)
			markdown_write_doc(&builder, item)
			builder_line(&builder, "```c")
			builder_format_line(&builder, "typedef %s %s;", field_string(item, "c_type"), field_string(item, "name"))
			builder_line(&builder, "```")
			builder_line(&builder)
		}
	}

	constants: [dynamic]^Node
	for item in field_items(manifest.root, "macro_constants")
	{
		if field_string(item, "header") == header
		{
			append(&constants, item)
		}
	}
	if len(constants) > 0
	{
		builder_line(&builder, "## Constants")
		builder_line(&builder)
		rows: [dynamic][3]string
		for item in constants
		{
			append(&rows, [3]string{
				fmt.aprintf("`%s`", field_string(item, "name")),
				fmt.aprintf("`%s`", field_string(item, "c_value")),
				field_string(item, "doc"),
			})
		}
		markdown_write_table(&builder, {"Constant", "Value", "Meaning"}, rows[:])
	}

	enums: [dynamic]^Node
	for item in field_items(manifest.root, "enum_constants")
	{
		if field_string(item, "header") == header
		{
			append(&enums, item)
		}
	}
	if len(enums) > 0
	{
		builder_line(&builder, "## Enumerated constants")
		builder_line(&builder)
		for item in enums
		{
			builder_format_line(&builder, "### `%s`", field_string(item, "type"))
			builder_line(&builder)
			markdown_write_doc(&builder, item)
			rows: [dynamic][3]string
			value_docs := node_field(item, "value_docs")
			for value in field_items(item, "values")
			{
				pair := node_items(value)
				name := node_string(pair[0])
				append(&rows, [3]string{
					fmt.aprintf("`%s`", name),
					fmt.aprintf("`%d`", node_integer(pair[1])),
					node_string(node_field(value_docs, name)),
				})
			}
			markdown_write_table(&builder, {"Constant", "Value", "Meaning"}, rows[:], right_column=1)
		}
	}

	callbacks: [dynamic]^Node
	for item in field_items(manifest.root, "callback_types")
	{
		if field_string(item, "header") == header
		{
			append(&callbacks, item)
		}
	}
	if len(callbacks) > 0
	{
		builder_line(&builder, "## Callback types")
		builder_line(&builder)
		for item in callbacks
		{
			builder_format_line(&builder, "### `%s`", field_string(item, "name"))
			builder_line(&builder)
			markdown_write_doc(&builder, item)
			params: [dynamic]string
			for parameter in field_items(item, "params")
			{
				append(&params, c_decl(parameter))
			}
			parameter_text := "void"
			if len(params) > 0
			{
				parameter_text = strings.join(params[:], ", ")
			}
			builder_line(&builder, "```c")
			builder_format_line(
				&builder,
				"typedef %s (ENTASIS_CALL *%s)(%s);",
				field_string(item, "return_c"),
				field_string(item, "name"),
				parameter_text,
			)
			builder_line(&builder, "```")
			builder_line(&builder)
			markdown_write_parameters(&builder, item)
			builder_format_line(&builder, "%s", field_string(item, "return_doc"))
			builder_line(&builder)
		}
	}

	structs: [dynamic]^Node
	for item in field_items(manifest.root, "structs")
	{
		if field_string(item, "header") == header
		{
			append(&structs, item)
		}
	}
	if len(structs) > 0
	{
		builder_line(&builder, "## Structures")
		builder_line(&builder)
		for item in structs
		{
			builder_format_line(&builder, "### `%s`", field_string(item, "name"))
			builder_line(&builder)
			markdown_write_doc(&builder, item)
			builder_format_line(&builder, "Size `%d`, alignment `%d`", field_integer(item, "size"), field_integer(item, "align"))
			builder_line(&builder)
			rows: [dynamic][3]string
			for field in field_items(item, "fields")
			{
				type_text := field_string(field, "c_type")
				count := node_field(field, "count")
				if count != nil
				{
					type_text = fmt.aprintf("%s[%d]", type_text, node_integer(count))
				}
				append(&rows, [3]string{
					fmt.aprintf("`%s`", field_string(field, "name")),
					fmt.aprintf("`%s`", type_text),
					field_string(field, "doc"),
				})
			}
			markdown_write_table(&builder, {"Field", "C type", "Meaning"}, rows[:])
		}
	}

	struct_aliases: [dynamic]^Node
	for item in field_items(manifest.root, "struct_aliases")
	{
		if field_string(item, "header") == header
		{
			append(&struct_aliases, item)
		}
	}
	if len(struct_aliases) > 0
	{
		builder_line(&builder, "## Struct aliases")
		builder_line(&builder)
		for item in struct_aliases
		{
			builder_format_line(&builder, "### `%s`", field_string(item, "name"))
			builder_line(&builder)
			markdown_write_doc(&builder, item)
			builder_line(&builder, "```c")
			builder_format_line(&builder, "typedef %s %s;", field_string(item, "target"), field_string(item, "name"))
			builder_line(&builder, "```")
			builder_line(&builder)
		}
	}

	functions: [dynamic]^Node
	for item in field_items(manifest.root, "functions")
	{
		if field_string(item, "header") == header
		{
			append(&functions, item)
		}
	}
	if len(functions) > 0
	{
		builder_line(&builder, "## Functions")
		builder_line(&builder)
		for item in functions
		{
			builder_format_line(&builder, "### `%s`", field_string(item, "name"))
			builder_line(&builder)
			markdown_write_doc(&builder, item)
			builder_line(&builder, "```c")
			builder_line(&builder, markdown_c_function_declaration(item))
			builder_line(&builder, "```")
			builder_line(&builder)
			markdown_write_parameters(&builder, item)
			builder_format_line(&builder, "%s", field_string(item, "return_doc"))
			builder_line(&builder)
		}
	}

	return builder_result(&builder)
}
