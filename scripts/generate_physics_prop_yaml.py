#!/usr/bin/env python3
import sys
import yaml
from collections import OrderedDict

def smart_cast(value: str):
    """Try to convert string to int or float, otherwise return as-is."""
    value = value.strip()
    if value == "":
        return None
    try:
        if "." in value or "e" in value.lower():
            return float(value)
        return int(value)
    except ValueError:
        return value

def parse_markdown_table(md_file, table_index):
    with open(md_file, "r") as f:
        lines = [line.rstrip("\n") for line in f]

    tables = []
    current = []
    inside = False
    for line in lines:
        if line.strip().startswith("|") and "---" not in line:  
            inside = True
            current.append(line.strip())
        elif "---" in line and inside:  
            current.append(line.strip())  # keep separator line
        else:
            if inside:
                tables.append(current)
                current = []
                inside = False
    if current:
        tables.append(current)

    if table_index >= len(tables):
        raise IndexError(f"No table at index {table_index}, found {len(tables)}")

    lines = tables[table_index]
    header = [h.strip() for h in lines[0].split("|")[1:-1]]

    rows = []
    for row in lines[2:]:  # skip header and separator
        if not row.startswith("|"):
            continue
        cols = [c.strip() for c in row.split("|")[1:-1]]
        if len(cols) == len(header):
            rows.append(dict(zip(header, cols)))
    return rows

def update_yaml(template_yaml, output_yaml, rows, column):
    with open(template_yaml, "r") as f:
        template_data = yaml.safe_load(f)

    # assume top-level key is "robot_physics"
    root_key = next(iter(template_data))
    section = template_data[root_key]

    for row in rows:
        attr = row.get("Attrs")
        if attr and column in row:
            section[attr] = smart_cast(row[column])

    with open(output_yaml, "w") as f:
        yaml.safe_dump(template_data, f, sort_keys=False)

    # print("Final data:", template_data)


if __name__ == "__main__":
    if len(sys.argv) != 6:
        print("Usage: python3 script.py <md_file> <table_index> <template_yaml> <output_yaml> <column>")
        sys.exit(1)

    md_file = sys.argv[1]
    table_number = int(sys.argv[2])
    template_yaml = sys.argv[3]
    output_yaml = sys.argv[4]
    column = sys.argv[5]

    rows = parse_markdown_table(md_file, table_number)
    update_yaml(template_yaml, output_yaml, rows, column)
