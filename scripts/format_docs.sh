#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (($# != 0)); then
  printf 'error: format_docs.sh does not accept arguments\n' >&2
  exit 2
fi

mapfile -d '' markdown_files < <(
  {
    find "$ROOT" -maxdepth 1 -type f -name '*.md' -print0
    find "$ROOT/docs" "$ROOT/sdk" "$ROOT/benchmarks" "$ROOT/results" -type f -name '*.md' -print0
  } | sort -z
)

changed_files=0
temporary_file=""
trap '[[ -z "$temporary_file" ]] || rm -f "$temporary_file"' EXIT

for markdown_file in "${markdown_files[@]}"; do
  temporary_file="$(mktemp "$ROOT/.format-docs.XXXXXX")"
  awk '
    function trim(value) {
      gsub(/^[ \t]+|[ \t]+$/, "", value)
      return value
    }

    function clear_array(values, key) {
      for (key in values) {
        delete values[key]
      }
    }

    function parse_row(line,    inner, position, character, run, code_ticks, backslashes, cursor, cell_count, buffer) {
      clear_array(parsed)
      line = trim(line)
      if (substr(line, 1, 1) != "|" || substr(line, length(line), 1) != "|") {
        return 0
      }

      inner = substr(line, 2, length(line) - 2)
      cell_count = 0
      buffer = ""
      code_ticks = 0
      position = 1
      while (position <= length(inner)) {
        character = substr(inner, position, 1)
        if (character == "`") {
          run = 1
          while (position + run <= length(inner) && substr(inner, position + run, 1) == "`") {
            run++
          }
          if (code_ticks == 0) {
            code_ticks = run
          } else if (code_ticks == run) {
            code_ticks = 0
          }
          buffer = buffer substr(inner, position, run)
          position += run
          continue
        }

        if (character == "|" && code_ticks == 0) {
          backslashes = 0
          cursor = position - 1
          while (cursor >= 1 && substr(inner, cursor, 1) == "\\") {
            backslashes++
            cursor--
          }
          if (backslashes % 2 == 0) {
            parsed[++cell_count] = trim(buffer)
            buffer = ""
            position++
            continue
          }
        }

        buffer = buffer character
        position++
      }
      parsed[++cell_count] = trim(buffer)
      return cell_count
    }

    function separator_mask(value,    mask) {
      value = trim(value)
      mask = 0
      if (substr(value, 1, 1) == ":") {
        mask += 1
        value = substr(value, 2)
      }
      if (substr(value, length(value), 1) == ":") {
        mask += 2
        value = substr(value, 1, length(value) - 1)
      }
      if (length(value) < 3 || value !~ /^-+$/) {
        return -1
      }
      return mask
    }

    function repeat_text(value, count,    result) {
      result = ""
      while (count-- > 0) {
        result = result value
      }
      return result
    }

    function padded_cell(value, width, mask,    remaining, before) {
      remaining = width - length(value)
      if (mask == 2) {
        return repeat_text(" ", remaining) value
      }
      if (mask == 3) {
        before = int(remaining / 2)
        return repeat_text(" ", before) value repeat_text(" ", remaining - before)
      }
      return value repeat_text(" ", remaining)
    }

    function separator_cell(width, mask,    colon_count) {
      colon_count = (mask == 1 || mask == 2) ? 1 : (mask == 3 ? 2 : 0)
      return (mask == 1 || mask == 3 ? ":" : "") \
        repeat_text("-", width - colon_count) \
        (mask == 2 || mask == 3 ? ":" : "")
    }

    function emit_table(start, end, column_count,    row, column, count, width, required, mask, output, indent) {
      clear_array(table)
      clear_array(widths)
      clear_array(alignments)

      for (row = start; row <= end; row++) {
        count = parse_row(lines[row])
        if (count != column_count) {
          printf "error: %s:%d: expected %d table cells, found %d\n", FILENAME, row, column_count, count > "/dev/stderr"
          exit 2
        }
        for (column = 1; column <= column_count; column++) {
          table[(row - start + 1) SUBSEP column] = parsed[column]
        }
      }

      for (column = 1; column <= column_count; column++) {
        mask = separator_mask(table[2 SUBSEP column])
        if (mask < 0) {
          printf "error: %s:%d: invalid table separator\n", FILENAME, start + 1 > "/dev/stderr"
          exit 2
        }
        alignments[column] = mask
        width = 0
        for (row = 1; row <= end - start + 1; row++) {
          if (row != 2 && length(table[row SUBSEP column]) > width) {
            width = length(table[row SUBSEP column])
          }
        }
        required = 3 + (mask == 1 || mask == 2 ? 1 : (mask == 3 ? 2 : 0))
        widths[column] = width > required ? width : required
      }

      match(lines[start], /^[ \t]*/)
      indent = substr(lines[start], 1, RLENGTH)
      for (row = 1; row <= end - start + 1; row++) {
        output = indent "| "
        for (column = 1; column <= column_count; column++) {
          if (row == 2) {
            output = output separator_cell(widths[column], alignments[column])
          } else {
            output = output padded_cell(table[row SUBSEP column], widths[column], alignments[column])
          }
          output = output (column == column_count ? " |" : " | ")
        }
        printf "%s%s", output, endings[start + row - 1]
      }
    }

    {
      line = $0
      if (sub(/\r$/, "", line)) {
        endings[NR] = "\r\n"
      } else {
        endings[NR] = "\n"
      }
      lines[NR] = line
    }

    END {
      line_number = 1
      while (line_number <= NR) {
        header_count = parse_row(lines[line_number])
        separator_count = line_number < NR ? parse_row(lines[line_number + 1]) : 0
        valid_separator = header_count > 0 && separator_count == header_count
        if (valid_separator) {
          for (column = 1; column <= separator_count; column++) {
            if (separator_mask(parsed[column]) < 0) {
              valid_separator = 0
              break
            }
          }
        }

        if (!valid_separator) {
          printf "%s%s", lines[line_number], endings[line_number]
          line_number++
          continue
        }

        table_end = line_number + 2
        while (table_end <= NR && parse_row(lines[table_end]) > 0) {
          table_end++
        }
        emit_table(line_number, table_end - 1, header_count)
        line_number = table_end
      }
    }
  ' "$markdown_file" > "$temporary_file"

  if cmp -s "$markdown_file" "$temporary_file"; then
    rm -f "$temporary_file"
    temporary_file=""
    continue
  fi

  chmod --reference="$markdown_file" "$temporary_file"
  mv -f "$temporary_file" "$markdown_file"
  temporary_file=""
  ((changed_files += 1))
done

printf 'FORMAT_DOCS_OK files=%d changed=%d\n' "${#markdown_files[@]}" "$changed_files"
