package main

import "core:fmt"
import "core:strings"

odin_identifier :: proc(name: string) -> string
{
	if name == "context" || name == "matrix"
	{
		return fmt.aprintf("%s_", name)
	}
	return name
}

odin_decl :: proc(field: ^Node) -> string
{
	odin_type := field_string(field, "odin_type")
	count := node_field(field, "count")
	if count != nil
	{
		odin_type = fmt.aprintf("[%d]%s", node_integer(count), odin_type)
	}
	return fmt.aprintf("%s: %s", odin_identifier(field_string(field, "name")), odin_type)
}

join_odin_params :: proc(item: ^Node) -> string
{
	params: [dynamic]string
	for parameter in field_items(item, "params")
	{
		append(&params, odin_decl(parameter))
	}
	return strings.join(params[:], ", ")
}

render_odin_types :: proc(manifest: ^Manifest, package_name: string) -> string
{
	builder := strings.builder_make_len_cap(0, 262144)
	builder_format_line(&builder, "// %s", BANNER_OWNER)
	builder_format_line(&builder, "package %s", package_name)
	builder_line(&builder)

	for alias in field_items(manifest.root, "scalar_aliases")
	{
		builder_format_line(&builder, "%s :: %s;", field_string(alias, "odin_name"), field_string(alias, "odin_type"))
	}
	builder_line(&builder)
	for constant in field_items(manifest.root, "macro_constants")
	{
		builder_format_line(&builder, "%s :: %s(%s);", field_string(constant, "odin_name"), field_string(constant, "odin_type"), field_string(constant, "odin_value"))
	}
	if len(field_items(manifest.root, "macro_constants")) > 0
	{
		builder_line(&builder)
	}

	for callback in field_items(manifest.root, "callback_types")
	{
		params := join_odin_params(callback)
		result := ""
		if field_string(callback, "return_odin") != ""
		{
			result = fmt.aprintf(" -> %s", field_string(callback, "return_odin"))
		}
		declaration := fmt.aprintf("%s :: #type proc \"c\" (%s)%s;", field_string(callback, "odin_name"), params, result)
		if len(declaration) <= 120
		{
			builder_line(&builder, declaration)
		}
		else
		{
			builder_format_line(&builder, "%s :: #type proc \"c\" (", field_string(callback, "odin_name"))
			for parameter in field_items(callback, "params")
			{
				builder_format_line(&builder, "\t%s,", odin_decl(parameter))
			}
			builder_format_line(&builder, ")%s;", result)
		}
	}
	builder_line(&builder)

	for item in field_items(manifest.root, "structs")
	{
		alignment := field_integer(item, "explicit_alignment")
		if alignment > 0
		{
			builder_format_line(&builder, "%s :: struct #align(%d)", field_string(item, "odin_name"), alignment)
		}
		else
		{
			builder_format_line(&builder, "%s :: struct", field_string(item, "odin_name"))
		}
		builder_line(&builder, "{")
		for field in field_items(item, "fields")
		{
			builder_format_line(&builder, "\t%s,", odin_decl(field))
		}
		builder_line(&builder, "}")
		builder_line(&builder)
	}
	for alias in field_items(manifest.root, "struct_aliases")
	{
		builder_format_line(&builder, "%s :: %s;", field_string(alias, "odin_name"), field_string(alias, "odin_target"))
	}
	builder_line(&builder)

	for item in field_items(manifest.root, "structs")
	{
		name := field_string(item, "odin_name")
		builder_format_line(&builder, "#assert(size_of(%s) == %d);", name, field_integer(item, "size"))
		builder_format_line(&builder, "#assert(align_of(%s) == %d);", name, field_integer(item, "align"))
		for offset in field_entries(item, "offsets")
		{
			assertion := fmt.aprintf("#assert(offset_of(%s, %s) == %d);", name, offset.key, node_integer(offset.value))
			if len(assertion) <= 120
			{
				builder_line(&builder, assertion)
			}
			else
			{
				builder_line(&builder, "#assert(")
				builder_format_line(&builder, "\toffset_of(%s, %s) == %d,", name, offset.key, node_integer(offset.value))
				builder_line(&builder, ");")
			}
		}
	}
	builder_line(&builder)
	return builder_result(&builder)
}

render_odin_exports :: proc(manifest: ^Manifest, library: string) -> string
{
	builder := strings.builder_make_len_cap(0, 524288)
	package_name := "entasis_c"
	if library == "cooking"
	{
		package_name = "entasis_cooking_c"
	}
	builder_format_line(&builder, "// %s", BANNER_OWNER)
	builder_format_line(&builder, "package %s", package_name)
	builder_line(&builder)

	for function in field_items(manifest.root, "functions")
	{
		if field_string(function, "library") != library
		{
			continue
		}
		name := field_string(function, "name")
		params := join_odin_params(function)
		result := ""
		if field_string(function, "return_odin") != ""
		{
			result = fmt.aprintf(" -> %s", field_string(function, "return_odin"))
		}
		args: [dynamic]string
		for parameter in field_items(function, "params")
		{
			append(&args, odin_identifier(field_string(parameter, "name")))
		}
		arguments := strings.join(args[:], ", ")
		builder_format_line(&builder, "@(export, link_name=\"%s\")", name)
		declaration := fmt.aprintf("export_%s :: proc \"c\" (%s)%s", name, params, result)
		if len(declaration) <= 120
		{
			builder_line(&builder, declaration)
		}
		else
		{
			builder_format_line(&builder, "export_%s :: proc \"c\" (", name)
			for parameter in field_items(function, "params")
			{
				builder_format_line(&builder, "\t%s,", odin_decl(parameter))
			}
			builder_format_line(&builder, ")%s", result)
		}
		builder_line(&builder, "{")
		prefix := ""
		if field_string(function, "return_odin") != ""
		{
			prefix = "return "
		}
		call := fmt.aprintf("%s%s(%s);", prefix, field_string(function, "impl"), arguments)
		if len(call) + 4 <= 120
		{
			builder_format_line(&builder, "\t%s", call)
		}
		else
		{
			builder_format_line(&builder, "\t%s%s(", prefix, field_string(function, "impl"))
			for parameter in field_items(function, "params")
			{
				builder_format_line(&builder, "\t\t%s,", odin_identifier(field_string(parameter, "name")))
			}
			builder_line(&builder, "\t);")
		}
		builder_line(&builder, "}")
		builder_line(&builder)
	}
	return builder_result(&builder)
}

render_odin_version :: proc(manifest: ^Manifest) -> string
{
	product := root_field(manifest, "product")
	return fmt.aprintf(`// %s
package entasis

// Version is the compile-time Entasis product version
Version :: struct
{{
	major:      u16,
	minor:      u16,
	patch:      u16,
	prerelease: string,
}}

// VERSION_MAJOR is the product major version
VERSION_MAJOR :: u16(%d);
// VERSION_MINOR is the product minor version
VERSION_MINOR :: u16(%d);
// VERSION_PATCH is the product patch version
VERSION_PATCH :: u16(%d);
// VERSION_PRERELEASE is empty for a stable release or contains the prerelease identifier
VERSION_PRERELEASE :: "%s";
// VERSION_STRING is the complete compile-time product version
VERSION_STRING :: "%s";

// ABI_VERSION identifies the public C ABI generation
ABI_VERSION :: u32(%d);

// CURRENT_VERSION contains the compile-time product version fields
CURRENT_VERSION :: Version{{
	major=VERSION_MAJOR,
	minor=VERSION_MINOR,
	patch=VERSION_PATCH,
	prerelease=VERSION_PRERELEASE,
}};

// version_current returns CURRENT_VERSION
version_current :: proc "contextless" () -> Version
{{
	return CURRENT_VERSION;
}}
`, BANNER_OWNER,
		field_integer(product, "version_major"), field_integer(product, "version_minor"),
		field_integer(product, "version_patch"), field_string(product, "version_prerelease"),
		field_string(product, "version_string"), field_integer(product, "abi_version"))
}
