function totalerror = GearedUR10Objective(X)
  %filename = 'gains.dat';
  %fileID = fopen(filename);
  %M = dlmread(filename,'\t');
  %M = textscan(fileID, '%s %d %d %d'); %read in contents of file
  %fclose(fileID);
  
  
  A = { 'base_gear_joint'      , ...
         'arm_gear_joint'      , ...
         'upperarm_gear_joint' , ...
         'fore_gear_joint'     , ...
         'wrist1_gear_joint'   , ...
         'wrist2_gear_joint'   , ...
         'l_finger_actuator'   , ...
         'r_finger_actuator'   };
  
  filename='gains.dat';
  fileID=fopen(filename,'wt'); %open file and clear
  [~,ncols] = size(A);      %get number of rows to iterate through cell structure

 for col = 1:ncols                         % write to new gains.dat file
     
     fprintf( fileID, '%s\t', A{1,col} ); % joint name
     fprintf( fileID, '%d\t', X(1,col)   ); % P gain
     fprintf( fileID, '%d\t', X(2,col)   ); % I gain
     fprintf( fileID, '%d\n', X(3,col)   ); % D gain   
 end
 
  fclose(fileID); % close file
  
  %make system call to run moby
  command = 'moby-driver -mt=3 -s=0.05 -p=/home/jshepherd/GearedUR10/release/libsinusoidal-controller.so ur10-geared.xml';
  
  system(command);
  %read from error txt file
  filename = 'ErrorPlot.txt';
  A = dlmread(filename);
 
  rownum = size(A);
  totalerror = 0;
  for row = 1:rownum
      totalerror = totalerror + A(row) * A(row);
  end
  
  %calculate total error from values in txt file
