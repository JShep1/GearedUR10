#!/bin/bash
cd ..
cd debug/
make
cd ..
cd gearedur10/
moby-driver -s=0.05  -mt=3 -p=/home/john/GearedUR10/debug/libsinusoidal-controller.so ur10-geared.xml
python plotError.py
