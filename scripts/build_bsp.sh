#!/bin/bash
# run this script from project main directory
export CXX=g++-5
export CC=gcc-5
rm -rf build/
mkdir build
cd build
cmake ..
make -j4
cd ..