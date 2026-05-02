#!/usr/bin/env bash
set -e

echo "[1/4] Build plugin"
mkdir -p build && cd build
cmake ..
make -j$(nproc)

echo "[2/4] Export plugin path"
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd):$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=$(pwd):$LD_LIBRARY_PATH

echo "[3/3] Done"
echo "Run this to persist:"
echo "echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd):\$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc"
echo "echo 'export LD_LIBRARY_PATH=$(pwd):\$LD_LIBRARY_PATH' >> ~/.bashrc"