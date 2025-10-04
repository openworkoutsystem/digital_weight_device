# Documentation Build Script

This directory contains the Sphinx documentation for the Open Workout System API.

## Quick Start

To build the documentation, simply run:

```batch
# Windows Batch (recommended)
build_docs.bat

# Or PowerShell
.\build_docs.ps1
```

This script will:
1. Install/update all required Python packages
2. Build the HTML documentation
3. Open the documentation in your default web browser

## Manual Build

If you prefer to build manually:

```batch
# Install requirements
pip install -r requirements.txt

# Build documentation
python -m sphinx.cmd.build -b html -W . build

# View results
start build/index.html
```

## Files

- `conf.py` - Sphinx configuration
- `index.rst` - Main documentation page
- `api.rst` - API reference
- `requirements.txt` - Python dependencies
- `build_docs.bat` - Automated build script (Windows Batch)
- `build_docs.ps1` - Automated build script (PowerShell)
- `Makefile` / `make.bat` - Alternative build system

The built documentation will be in the `build/` directory.