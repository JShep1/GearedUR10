#!/bin/bash
cd ..
cd debug/
make
cd ..
cd gearedur10/
moby-driver -s=0.05  -mt=3 -p=/home/john/GearedUR10/debug/libsinusoidal-controller.so ur10-geared.xml
python plot_PD_PID.py base_gear_joint
python plot_PD_PID.py arm_gear_joint
python plot_PD_PID.py upperarm_gear_joint
python plot_PD_PID.py fore_gear_joint
python plot_PD_PID.py wrist1_gear_joint
python plot_PD_PID.py wrist2_gear_joint
python plotError.py
