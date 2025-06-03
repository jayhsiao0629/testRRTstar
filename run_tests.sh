#!/bin/bash

echo "=== RRT* Unit Test Runner ==="
echo "Preparing test environment..."

# Create necessary directories
mkdir -p Mfiles
mkdir -p bin

# Check if main project is compiled
if [ ! -f "bin/RRTSTAR" ]; then
    echo "Main project not compiled. Compiling now..."
    make
    if [ $? -ne 0 ]; then
        echo "Failed to compile main project. Please check your Makefile and dependencies."
        exit 1
    fi
fi

# Compile test
echo "Compiling test..."
g++ -std=c++17 -Wall -Wextra -o test_main test_main.cpp

if [ $? -ne 0 ]; then
    echo "Failed to compile test. Please check for compilation errors."
    exit 1
fi

# Run test
echo "Running tests..."
./test_main

# Clean up
echo "Cleaning up test files..."
rm -f test_main
rm -f test_output*.txt

echo "Test completed!"