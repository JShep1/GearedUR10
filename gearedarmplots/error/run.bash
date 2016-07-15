#!/bin/bash

rm wrist1/*.py
rm wrist2/*.py
rm wrist3/*.py
rm elbow/*.py
rm shoulderlift/*.py
rm shoulderpan/*.py

cp plotGearJoints.py elbow/
cp plotGearJoints.py shoulderlift/
cp plotGearJoints.py shoulderpan/
cp plotGearJoints.py wrist1/
cp plotGearJoints.py wrist2/
cp plotGearJoints.py wrist3/

cd elbow/
python plotGearJoints.py elbow
cp elbow_error.png ..
cd ..

cd shoulderlift/
python plotGearJoints.py shoulderlift
cp shoulderlift_error.png ..
cd ..

cd shoulderpan/
python plotGearJoints.py shoulderpan
cp shoulderpan_error.png ..
cd ..

cd wrist1/
python plotGearJoints.py wrist_1
cp wrist_1_error.png ..
cd ..

cd wrist2/
python plotGearJoints.py wrist_2
cp wrist_2_error.png ..
cd ..

cd wrist3/
python plotGearJoints.py wrist_3
cp wrist_3_error.png ..
cd ..


rm allplots/*.png
mv *.png allplots/



