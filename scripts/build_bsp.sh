#!/bin/bash
# run this script from project main directory

rm -rf build/
mkdir build
cd build
cmake ..
make -j4
cd ..