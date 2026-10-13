#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "======================================================="
echo "Running Candidate Reverse Crawl Specification Suite"
echo "======================================================="
g++ -O2 -std=c++17 \
    -I"${PACKAGE_ROOT}/production_source" \
    -I"${SCRIPT_DIR}/mock_include" \
    "${SCRIPT_DIR}/test_reverse_crawl_candidate.cpp" \
    -o "${SCRIPT_DIR}/test_reverse_crawl_candidate"
"${SCRIPT_DIR}/test_reverse_crawl_candidate"

echo ""
echo "======================================================="
echo "Running Open-Loop Safety Expiry & Arbitration Suite"
echo "======================================================="
g++ -O2 -std=c++17 \
    -I"${PACKAGE_ROOT}/production_source" \
    -I"${SCRIPT_DIR}/mock_include" \
    "${SCRIPT_DIR}/test_open_loop_expiry.cpp" \
    -o "${SCRIPT_DIR}/test_open_loop_expiry"
"${SCRIPT_DIR}/test_open_loop_expiry"

echo ""
echo "======================================================="
echo "Running Case 2 Behavioral Validation Suite"
echo "======================================================="
g++ -O2 -std=c++17 \
    -I"${PACKAGE_ROOT}/production_source" \
    -I"${SCRIPT_DIR}/mock_include" \
    "${SCRIPT_DIR}/test_case2_behavioral.cpp" \
    -o "${SCRIPT_DIR}/test_case2_behavioral"
"${SCRIPT_DIR}/test_case2_behavioral"

echo ""
echo "======================================================="
echo "Running WheelController Regression Suite (Tests 1-24)"
echo "======================================================="
g++ -O2 -std=c++17 \
    -I"${PACKAGE_ROOT}/production_source" \
    -I"${SCRIPT_DIR}/mock_include" \
    -DTEST_PATCHED \
    "${SCRIPT_DIR}/test_wheel_controller_native.cpp" \
    -o "${SCRIPT_DIR}/test_wheel_controller_native"
"${SCRIPT_DIR}/test_wheel_controller_native"

echo ""
echo "======================================================="
echo "ALL TEST SUITES COMPILED AND PASSED SUCCESSFULLY!"
echo "======================================================="
