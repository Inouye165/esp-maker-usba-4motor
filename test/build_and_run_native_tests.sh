#!/usr/bin/env bash
set -e

# test/build_and_run_native_tests.sh
# Compiles and executes the native C++ test suite for WheelController regressions.
# Requires: g++ with C++17 support.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "======================================================="
echo "Building Native C++ Drivetrain Regression Test Suite"
echo "======================================================="

cd "${REPO_DIR}"
g++ -O2 -std=c++17 \
    -Iinclude \
    -Isrc \
    -Itest \
    -Itest/mock_include \
    test/test_wheel_controller_native.cpp \
    -DTEST_PATCHED \
    -o test_patched_native

echo "Build successful: ./test_patched_native"
echo "Executing test runner..."
echo "======================================================="

# If arguments passed, pass them directly (e.g. ./test/build_and_run_native_tests.sh 19)
./test_patched_native "$@"
