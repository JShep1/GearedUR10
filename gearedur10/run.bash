#!/bin/bash
cd ..
cd release/
make
cd ..
cd gearedur10/
moby-driver -s=0.05 -mt=3 -r -oi -p=/home/jshepherd/GearedUR10/release/libsinusoidal-controller.so ur10-geared.xml
python plot_PD_PID.py base_gear_joint
python plot_PD_PID.py arm_gear_joint
python plot_PD_PID.py upperarm_gear_joint
python plot_PD_PID.py fore_gear_joint
python plot_PD_PID.py wrist1_gear_joint
python plot_PD_PID.py wrist2_gear_joint
python plotError.py
python plotVelocities.py base_gear_joint
python plotVelocities.py arm_gear_joint
python plotVelocities.py upperarm_gear_joint
python plotVelocities.py fore_gear_joint
python plotVelocities.py wrist1_gear_joint
python plotVelocities.py wrist2_gear_joint
python plotControl.py base_gear_joint
python plotControl.py arm_gear_joint
python plotControl.py upperarm_gear_joint
python plotControl.py fore_gear_joint
python plotControl.py wrist1_gear_joint
python plotControl.py wrist2_gear_joint
