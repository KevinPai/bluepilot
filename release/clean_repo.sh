#!/bin/bash

echo "Starting cleanup..."

# Debug output for __pycache__ directories
echo "Searching for __pycache__ directories..."
find . -type d -name '__pycache__' -print -exec rm -rf {} +

# Debug output for .DS_Store files
echo "Searching for .DS_Store files..."
find . -type f -name '.DS_Store' -print -exec rm -f {} +

# Debug output for .moc files
echo "Searching for .moc files..."
find . -type f -name '*.moc' -print -exec rm -f {} +

# Remove all _generated.dbc files
echo "Searching for _generated.dbc files..."
find . -type f -name '*_generated.dbc' -exec rm -f {} +

# Debug output for .o files
echo "Searching for .o files..."
find . -type f -name '*.o' -print -exec rm -f {} +

# remove all files that start with moc_
echo "Searching for moc_ files..."
find . -type f -name 'moc_*' -print -exec rm -f {} +

# Remove all .tmp files
echo "Searching for .tmp files..."
find . -type f -name '*.tmp' -print -exec rm -f {} +

# Clean other directories
echo "Cleaning other directories..."
rm -rf cereal/gen
rm -rf .venv
rm -rf .mypy_cache
rm -rf panda/board/jungle/obj
rm -f panda/board/obj/*.h
rm -f panda/board/obj/version
rm -f selfdrive/controls/lib/longitudinal_mpc_lib/*.json
rm -f selfdrive/controls/lib/lateral_mpc_lib/*.json
rm -rf tools/plotjuggler/bin

rm -f selfdrive/assets/translations_assets.qrc

rm -f opendbc_repo/opendbc/safety/tests/misra/checkers.txt
rm -f opendbc_repo/opendbc/safety/tests/misra/suppressions.txt

# Clean up empty directories named generated
echo "Cleaning up empty directories named generated..."
find . -type d -name 'generated' -empty -print -exec rm -rf {} +

echo "Cleanup complete."
