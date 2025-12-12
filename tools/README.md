# Development Tools

This directory contains utilities for maintaining code quality and consistency.

## validate_set_to_read_map.py

**Purpose:** Validates that the hand-coded `setToReadMap` in `CommandDispatcher.cpp` stays synchronized with the JSON specification (`spec/ts590sg_cat_commands_v3.json`), which is the single source of truth.

**Usage:**
```bash
python3 tools/validate_set_to_read_map.py
```

**Exit Codes:**
- `0` - Validation passed (all formats match)
- `1` - Validation failed (format mismatches detected)
- `2` - File not found or parsing error

**Features:**
- Automatically expands const parameters (e.g., `AGP1;` with P1=const(0) becomes `AG0;`)
- Handles parameters with min==max (effectively const)
- Reports INFO for commands not in map (expected - not all need auto-query)
- Reports WARNING for commands in map but not in JSON (verify intentional)
- Only fails on format mismatches (actual errors)

**Integration:**
Recommended to run as part of CI/CD or pre-commit hooks to catch synchronization issues early.

**Example Output:**
```
✅ Validation PASSED - All setToReadMap formats match JSON spec
   Note: Review EXTRA commands warning above
```

## Future Tools

Additional tools can be added here for:
- Code generation from JSON spec
- CAT command test generation
- Performance profiling helpers
- Static analysis utilities
