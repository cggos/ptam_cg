#!/bin/sh

cd ../

mkdir build
cd build

cmake ..

time make -j2
