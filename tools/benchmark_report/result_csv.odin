package benchmark_report

import "core:bufio"
import "core:encoding/csv"
import "core:mem"
import "core:os"

Result_CSV :: struct
{
	storage: []u8,
	records: [][]string,
}

result_budget_error :: proc(required, available: u64) -> Report_Result
{
	return {status=.Budget_Exceeded, required_bytes=required, available_bytes=available};
}

result_csv_read :: proc(path: string, budget: u64) -> (output: Result_CSV, result: Report_Result)
{
	file, open_error := os.open(path);
	if open_error != nil
	{
		return {}, report_error(.File_Read_Failed, "failed to open %s", path);
	}
	defer os.close(file);
	file_size, size_error := os.file_size(file);
	if size_error != nil || file_size < 0 || u64(file_size) > u64(max(int))
	{
		return {}, report_error(.File_Read_Failed, "invalid file size: %s", path);
	}
	if u64(file_size) > budget || u64(file_size) > (max(u64)-8192)/64
	{
		return {}, result_budget_error(u64(file_size), budget);
	}
	bytes, allocation_error := make([]u8, int(file_size));
	if allocation_error != nil
	{
		return {}, {status=.Allocation_Failed};
	}
	defer delete(bytes);
	read, read_error := os.read_at(file, bytes, 0);
	if read_error != nil || read != len(bytes)
	{
		return {}, report_error(.File_Read_Failed, "failed to read %s", path);
	}
	rows, columns: u64 = 1, 1;
	for byte in bytes
	{
		if byte == '\n'
		{
			rows += 1;
		}
		else if byte == ','
		{
			columns += 1;
		}
	}
	// Delimiter counts are upper bounds, not a second CSV parser. Quoted commas
	// and newlines only overreserve; the standard reader owns interpretation.
	fields: u64 = rows+columns;
	character_capacity: u64 = max(u64(len(bytes)), u64(csv.DEFAULT_RECORD_BUFFER_CAPACITY));
	// input, raw/record scratch, copied field bytes, row/field views, and reader buffers
	required: u64 = u64(len(bytes))*2+character_capacity*2+
		rows*size_of([]string)+fields*size_of(string)+columns*(size_of(int)+size_of(string))+
		bufio.DEFAULT_BUF_SIZE+16*8;
	if required > budget || required > u64(max(int))
	{
		return {}, result_budget_error(required, budget);
	}
	storage: []u8;
	storage, allocation_error = make([]u8, int(required)-len(bytes));
	if allocation_error != nil
	{
		return {}, {status=.Allocation_Failed};
	}
	defer if result.status != .Ok
	{
		delete(storage);
	}
	arena: mem.Arena;
	mem.arena_init(&arena, storage);
	allocator: mem.Allocator = mem.arena_allocator(&arena);
	reader: csv.Reader;
	reader.raw_buffer = make([dynamic]u8, 0, int(character_capacity), allocator);
	reader.record_buffer = make([dynamic]u8, 0, int(character_capacity), allocator);
	reader.field_indices = make([dynamic]int, 0, int(columns), allocator);
	reader.last_record = make([dynamic]string, 0, int(columns), allocator);
	reader.reuse_record, reader.reuse_record_buffer = true, true;
	csv.reader_init_with_string(&reader, string(bytes), allocator);
	defer csv.reader_destroy(&reader);
	records: [][]string = make([][]string, int(rows), allocator);
	field_storage: []string = make([]string, int(fields), allocator);
	characters: []u8 = make([]u8, len(bytes), allocator);
	row, field, offset: int;
	for
	{
		record, error := csv.read(&reader, allocator);
		if csv.is_io_error(error, .EOF)
		{
			break;
		}
		if error != nil
		{
			return {}, report_error(.Csv_Invalid, "invalid CSV: %s", path);
		}
		start: int = field;
		for text in record
		{
			copy(characters[offset:], text);
			field_storage[field] = string(characters[offset:offset+len(text)]);
			field += 1;
			offset += len(text);
		}
		records[row] = field_storage[start:field];
		row += 1;
	}
	return {storage, records[:row]}, {status=.Ok};
}
