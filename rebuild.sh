#!/bin/bash

# Script to clean directory, run cmake with specific flags, and build with make

# Exit on any error
set -e

# Remove all files and directories in the current directory
echo "Cleaning current directory..."
rm -rf ./*

# Run cmake with specified build options
echo "Running cmake..."
cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=on -DSEND_REMOTE=OFF

# Build with make using 20 parallel jobs
echo "Building with make..."
make -j20

echo "Build completed successfully!"