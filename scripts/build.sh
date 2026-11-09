#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")/.."
mkdir -p build
${CXX:-g++} -std=c++17 -O2 -Wall -Wextra -Werror -shared -fPIC \
  -Ivendor/Hexapod-Simulation/Hexapod-Firmware/src src/bridge.cpp -o build/libmuha_fw.so
