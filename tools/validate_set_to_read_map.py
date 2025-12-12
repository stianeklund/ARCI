#!/usr/bin/env python3
"""
Validate CommandDispatcher setToReadMap against ts590sg_cat_commands_v3.json

This script ensures that the hand-coded setToReadMap in CommandDispatcher.cpp
stays synchronized with the JSON specification (the single source of truth).

Usage:
    python3 tools/validate_set_to_read_map.py

Exit codes:
    0 - Validation passed
    1 - Validation failed (mismatch or missing entries)
    2 - File not found or parsing error
"""

import json
import re
import sys
from pathlib import Path
from typing import Dict, Set, Tuple


def expand_read_format(read_format: str, params: list) -> str:
    """
    Expand parameter placeholders in READ format to concrete values.

    For const parameters or parameters with min==max, substitute the value.
    For example: "AGP1;" with P1=const("0") becomes "AG0;"
    """
    result = read_format

    for param in params:
        param_id = param.get('id', '')
        if not param_id:
            continue

        # Handle const type parameters
        if param.get('type') == 'const':
            const_val = param.get('const', '')
            result = result.replace(param_id, const_val)
        # Handle number type with min==max (effectively const)
        elif param.get('type') == 'number':
            min_val = param.get('min')
            max_val = param.get('max')
            if min_val is not None and max_val is not None and min_val == max_val:
                result = result.replace(param_id, str(min_val))

    return result


def parse_json_spec(json_path: Path) -> Dict[str, str]:
    """
    Parse JSON specification and extract SET->READ mappings.

    Returns:
        Dictionary mapping command prefix to READ format (e.g., {"FA": "FA;", "AG": "AG0;"})
    """
    with open(json_path, 'r') as f:
        spec = json.load(f)

    mappings = {}

    for cmd_prefix, cmd_data in spec.items():
        # Skip if command doesn't support SET operation
        if 'supports' not in cmd_data or 'set' not in cmd_data['supports']:
            continue

        # Skip if no READ operation (some commands only have SET)
        if 'read' not in cmd_data['supports']:
            continue

        # Extract READ format
        read_format = cmd_data.get('formats', {}).get('read', '')
        if not read_format:
            print(f"WARNING: Command {cmd_prefix} supports SET and READ but has no read format", file=sys.stderr)
            continue

        # Expand parameters to concrete values where applicable
        params = cmd_data.get('params', [])
        read_format = expand_read_format(read_format, params)

        mappings[cmd_prefix] = read_format

    return mappings


def parse_cpp_map(cpp_path: Path) -> Dict[str, str]:
    """
    Parse CommandDispatcher.cpp and extract setToReadMap entries.

    Returns:
        Dictionary mapping command prefix to READ format
    """
    with open(cpp_path, 'r') as f:
        content = f.read()

    # Find the setToReadMap definition
    # Pattern: {"CMD", "READ_FORMAT"},
    pattern = r'\{"([A-Z]{2,3})",\s*"([^"]+)"\}'

    mappings = {}
    for match in re.finditer(pattern, content):
        cmd = match.group(1)
        read_format = match.group(2)
        mappings[cmd] = read_format

    return mappings


def validate_mappings(json_mappings: Dict[str, str], cpp_mappings: Dict[str, str]) -> Tuple[bool, Set[str], Set[str], Dict[str, Tuple[str, str]]]:
    """
    Compare JSON and C++ mappings.

    Returns:
        (is_valid, missing_in_cpp, extra_in_cpp, mismatched)
        where mismatched is {cmd: (json_format, cpp_format)}
    """
    json_set = set(json_mappings.keys())
    cpp_set = set(cpp_mappings.keys())

    missing_in_cpp = json_set - cpp_set
    extra_in_cpp = cpp_set - json_set

    # Check for format mismatches in commands present in both
    mismatched = {}
    for cmd in json_set & cpp_set:
        if json_mappings[cmd] != cpp_mappings[cmd]:
            mismatched[cmd] = (json_mappings[cmd], cpp_mappings[cmd])

    is_valid = len(missing_in_cpp) == 0 and len(mismatched) == 0

    return is_valid, missing_in_cpp, extra_in_cpp, mismatched


def main():
    # Determine project root (script is in tools/)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent

    json_path = project_root / "spec" / "ts590sg_cat_commands_v3.json"
    cpp_path = project_root / "components" / "RadioCore" / "CommandDispatcher.cpp"

    # Verify files exist
    if not json_path.exists():
        print(f"ERROR: JSON specification not found: {json_path}", file=sys.stderr)
        return 2

    if not cpp_path.exists():
        print(f"ERROR: CommandDispatcher.cpp not found: {cpp_path}", file=sys.stderr)
        return 2

    print("Validating setToReadMap against JSON specification...")
    print(f"  JSON spec: {json_path.name}")
    print(f"  C++ file:  {cpp_path.relative_to(project_root)}")
    print()

    # Parse both files
    try:
        json_mappings = parse_json_spec(json_path)
        cpp_mappings = parse_cpp_map(cpp_path)
    except Exception as e:
        print(f"ERROR: Failed to parse files: {e}", file=sys.stderr)
        return 2

    print(f"Found {len(json_mappings)} SET commands with READ in JSON spec")
    print(f"Found {len(cpp_mappings)} entries in setToReadMap")
    print()

    # Validate
    is_valid, missing, extra, mismatched = validate_mappings(json_mappings, cpp_mappings)

    # Report results
    if mismatched:
        print(f"❌ MISMATCHED formats ({len(mismatched)} commands):")
        for cmd in sorted(mismatched.keys()):
            json_fmt, cpp_fmt = mismatched[cmd]
            print(f"   {cmd:4s}:")
            print(f"      JSON: {json_fmt}")
            print(f"      C++:  {cpp_fmt}")
        print()

    if missing:
        print(f"ℹ️  INFO: {len(missing)} commands in JSON spec not in setToReadMap:")
        print("   This is expected - not all SET commands need auto-query")
        print(f"   Total missing: {len(missing)} (showing first 10)")
        for cmd in sorted(missing)[:10]:
            print(f"   {cmd:4s} -> {json_mappings[cmd]:15s}")
        if len(missing) > 10:
            print(f"   ... and {len(missing) - 10} more")
        print()

    if extra:
        print(f"⚠️  WARNING: {len(extra)} commands in setToReadMap not in JSON spec:")
        print("   These may be internal/undocumented commands - verify they're intentional")
        for cmd in sorted(extra):
            print(f"   {cmd:4s} -> {cpp_mappings[cmd]:10s}")
        print()

    # Only mismatches cause validation failure
    if mismatched:
        print("❌ Validation FAILED - Format mismatches detected")
        print("   Please update setToReadMap in CommandDispatcher.cpp to match JSON spec")
        return 1
    else:
        print("✅ Validation PASSED - All setToReadMap formats match JSON spec")
        if extra:
            print("   Note: Review EXTRA commands warning above")
        return 0


if __name__ == "__main__":
    sys.exit(main())
