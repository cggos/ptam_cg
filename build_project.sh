#!/bin/sh

mkdir build
cd build

cmake ..

time make -j2
