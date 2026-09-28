package main

import "core:encoding/json"
import "core:fmt"
import "core:os"
import "core:strings"

Asset :: struct
{
	name:   string,
	url:    string,
	digest: string,
	size:   i64,
}

fail :: proc(message: string)
{
	fmt.eprintln(message)
	os.exit(1)
}

parse_object :: proc(data: []byte) -> json.Object
{
	value: json.Value
	parse_error := json.unmarshal(data, &value, .JSON, context.temp_allocator)
	if parse_error != nil
	{
		fail(fmt.tprintf("Invalid release metadata: %v", parse_error))
	}
	object, object_ok := value.(json.Object)
	if !object_ok
	{
		fail("Expected a release metadata object")
	}
	return object
}

required_text :: proc(object: json.Object, key: string) -> string
{
	value, value_ok := object[key].(json.String)
	if !value_ok || len(value) == 0 || strings.contains(value, "\n") || strings.contains(value, "\r")
	{
		fail(fmt.tprintf("Missing or invalid release field: %s", key))
	}
	return value
}

require_hex :: proc(value: string, length: int)
{
	if len(value) != length
	{
		fail("Invalid published digest or commit")
	}
	for character in value
	{
		if !(character >= '0' && character <= '9' || character >= 'a' && character <= 'f')
		{
			fail("Invalid published digest or commit")
		}
	}
}

main :: proc()
{
	defer free_all(context.temp_allocator)
	if len(os.args) != 3
	{
		fmt.eprintln("usage: update_release TOOL RELEASE_JSON")
		os.exit(2)
	}
	tool := os.args[1]
	repository, prefix := "", ""
	switch tool
	{
	case "odin": repository = "odin-lang/Odin"
	case "llvm": repository, prefix = "llvm/llvm-project", "llvmorg-"
	case "cmake": repository, prefix = "Kitware/CMake", "v"
	case "ninja": repository, prefix = "ninja-build/ninja", "v"
	case:
		fmt.eprintln("Unknown release tool")
		os.exit(2)
	}
	data, read_error := os.read_entire_file(os.args[2], context.temp_allocator)
	if read_error != nil
	{
		fail(fmt.tprintf("Cannot read release metadata: %s", os.args[2]))
	}
	release := parse_object(data)
	flag_fields := [2]string{"draft", "prerelease"}
	for key in flag_fields
	{
		flag, flag_ok := release[key].(json.Boolean)
		if !flag_ok || flag
		{
			fail("Only published stable/monthly releases are supported")
		}
	}
	tag := required_text(release, "tag_name")
	if !strings.has_prefix(tag, prefix)
	{
		fail("Unexpected release tag")
	}
	version := tag[len(prefix):]
	names: [2]string
	switch tool
	{
	case "odin": names = {fmt.tprintf("odin-linux-amd64-%s.tar.gz", tag), fmt.tprintf("odin-windows-amd64-%s.zip", tag)}
	case "llvm": names = {fmt.tprintf("LLVM-%s-Linux-X64.tar.xz", version), fmt.tprintf("clang+llvm-%s-x86_64-pc-windows-msvc.tar.xz", version)}
	case "cmake": names = {fmt.tprintf("cmake-%s-linux-x86_64.tar.gz", version), fmt.tprintf("cmake-%s-windows-x86_64.zip", version)}
	case "ninja": names = {"ninja-linux.zip", "ninja-win.zip"}
	}
	assets, assets_ok := release["assets"].(json.Array)
	if !assets_ok
	{
		fail("Missing release assets")
	}
	selected: [2]Asset
	for name, index in names
	{
		matches := 0
		for value in assets
		{
			asset, asset_ok := value.(json.Object)
			if !asset_ok
			{
				fail("Invalid release asset")
			}
			if required_text(asset, "name") != name
			{
				continue
			}
			matches += 1
			digest := required_text(asset, "digest")
			if !strings.has_prefix(digest, "sha256:")
			{
				fail("Published SHA-256 unavailable")
			}
			require_hex(digest[7:], 64)
			url := required_text(asset, "browser_download_url")
			if !strings.has_prefix(url, fmt.tprintf("https://github.com/%s/releases/download/", repository))
			{
				fail("Unexpected official release URL")
			}
			size, size_ok := asset["size"].(json.Integer)
			if !size_ok || size <= 0
			{
				fail("Invalid release asset size")
			}
			selected[index] = Asset{name = name, url = url, digest = digest[7:], size = size}
		}
		if matches != 1
		{
			fail(fmt.tprintf("Required official asset unavailable or ambiguous: %s", name))
		}
	}
	commit := ""
	if tool == "odin"
	{
		url := fmt.tprintf("https://api.github.com/repos/%s/commits/%s", repository, tag)
		fmt.eprintf("RELEASE_METADATA %s\n", url)
		command := []string{
			"curl", "--fail", "--silent", "--show-error", "--location",
			"--connect-timeout", "30", "--max-time", "30",
			"-H", "Accept: application/vnd.github+json", "-H", "User-Agent: Entasis-toolchains",
			"--url", url,
		}
		state, stdout, stderr, process_error := os.process_exec(os.Process_Desc{command = command}, context.temp_allocator)
		if process_error != nil || !state.exited || state.exit_code != 0
		{
			fmt.eprint(string(stderr))
			fail(fmt.tprintf("Cannot obtain the release commit: %v", process_error))
		}
		commit = required_text(parse_object(stdout), "sha")
		require_hex(commit, 40)
	}
	fmt.printf("%s.repository=%s\n%s.release=%s\n%s.version=%s\n", tool, repository, tool, tag, tool, version)
	if len(commit) > 0
	{
		fmt.printf("%s.commit=%s\n", tool, commit)
	}
	hosts := [2]string{"linux-amd64", "windows-amd64"}
	for host, index in hosts
	{
		asset := selected[index]
		fmt.printf("%s.hosts.%s.asset=%s\n", tool, host, asset.name)
		fmt.printf("%s.hosts.%s.url=%s\n", tool, host, asset.url)
		fmt.printf("%s.hosts.%s.sha256=%s\n", tool, host, asset.digest)
		fmt.printf("%s.hosts.%s.size=%d\n", tool, host, asset.size)
	}
}
