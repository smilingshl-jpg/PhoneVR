#!/usr/bin/env bash
# Builds and runs the host-side native tests (pose maths). Needs any C++17 compiler (g++ or clang++).
set -euo pipefail
cd "$(dirname "$0")"
CXX="${CXX:-g++}"
out="$(mktemp -d)"
for t in test_*.cpp; do
    "$CXX" -std=c++17 -O1 -Wall -Wextra -o "$out/${t%.cpp}" "$t"
    "$out/${t%.cpp}"
done
