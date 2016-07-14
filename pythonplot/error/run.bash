#!/bin/bash
cd 01/
rm *desiredPID.txt
mv arm_gear_joint_statePID.txt shoulderlift01.txt
mv base_gear_joint_statePID.txt shoulderpan01.txt
mv fore_gear_joint_statePID.txt wrist_1_01.txt
mv upperarm_gear_joint_statePID.txt elbow01.txt
mv wrist1_gear_joint_statePID.txt wrist_2_01.txt
mv wrist2_gear_joint_statePID.txt wrist_3_01.txt

mv shoulderlift01.txt ..
mv shoulderpan01.txt ..
mv wrist_1_01.txt ..
mv elbow01.txt ..
mv wrist_2_01.txt ..
mv wrist_3_01.txt ..
cd ..
mv shoulderlift01.txt shoulderlift/
mv shoulderpan01.txt shoulderpan/
mv wrist_1_01.txt wrist1/
mv elbow01.txt elbow/
mv wrist_2_01.txt wrist2/
mv wrist_3_01.txt wrist3/

cd 05/
rm *desiredPID.txt
mv arm_gear_joint_statePID.txt shoulderlift05.txt
mv base_gear_joint_statePID.txt shoulderpan05.txt
mv fore_gear_joint_statePID.txt wrist_1_05.txt
mv upperarm_gear_joint_statePID.txt elbow05.txt
mv wrist1_gear_joint_statePID.txt wrist_2_05.txt
mv wrist2_gear_joint_statePID.txt wrist_3_05.txt

mv shoulderlift05.txt ..
mv shoulderpan05.txt ..
mv wrist_1_05.txt ..
mv elbow05.txt ..
mv wrist_2_05.txt ..
mv wrist_3_05.txt ..

cd ..

mv shoulderlift05.txt shoulderlift/
mv shoulderpan05.txt shoulderpan/
mv wrist_1_05.txt wrist1/
mv elbow05.txt elbow/
mv wrist_2_05.txt wrist2/
mv wrist_3_05.txt wrist3/


