#!/bin/bash

set -e

mkdir -p src
cd src

test -d octomap || git clone https://github.com/OctoMap/octomap.git
cd octomap

git checkout v1.8.0

mkdir -p build
cd build

cmake ..
make -j
