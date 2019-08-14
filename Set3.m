clear all
close all
clc
format long g

%% User Input Block: (move to the top of the file after completion)

test_name='Initial Program Test 1';
target_size='2.4384m x 2.4384m';
distance=1; %Distance to target in meters
temperature= 74; %Degrees i n Farenheit 
humidity= 45; %Humidity percentage
ambient_light= 300; %Ambient light in lx

target_notes='Target Reflectivity: White (88%)'; %Reflectivity or other notes here 

%%%%%%%%% Setting Entry %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
itn=1; %Number of iterations of grabbing data from sensor 

TLx=0; %Default 0
TLy=15; %Default 15
BRx=15; %Default 15
BRy=0; %Default 0

set_char1='J';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

space1=[' ',' '];
space2=[' ',' ';' ',' '];
colnum=10; 
column_titles={'Ultrasonic Sensor Range (mm), Optical Sensor Range (mm), ' };
total_time=0; 
delay=2.5; %Time delay between iterations to give microcontroller time to iterate

% Sets proper format of string to be sent to the microcontroller

if TLx>=0 &&  TLx<=9
 ch_TLx=strcat('0',int2str(TLx)); 
 
else
    ch_TLx=int2str(TLx);
    
end

if TLy>=0 &&  TLy<=9
 ch_TLy=strcat('0',int2str(TLy)); 
 
else
    ch_TLy=int2str(TLy);
      
end

if BRx>=0 &&  BRx<=9
 ch_BRx=strcat('0',int2str(BRx)); 
   
else
    ch_BRx=int2str(BRx);
    
end

if BRy>=0 &&  BRy<=9
 ch_BRy=strcat('0',int2str(BRy)); 

else
    ch_BRy=int2str(BRy);
       
end

if itn>=0 &&  itn<=9
 ch_itn=strcat('0','0','0',int2str(itn)); 

elseif itn>=10 &&  itn<=99
    ch_itn=strcat('0','0',int2str(itn)); 
        
elseif itn>=100 &&  itn<=999
     ch_itn=strcat('0',int2str(itn)); 
     
else
    ch_itn=int2str(itn); 
       
end






%% Iteration 1 %%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
itn_num=1;


%%%%%%%%%%%%%%%%
i=1; 
itn_matrix=zeros(itn,1); 

while(i<((itn*52)*2)+1)
    
    itn_matrix(i,1)=itn_num; 
    i=i+1;
   
end


set1=strcat(set_char1,ch_TLx,ch_TLy,ch_BRx,ch_BRy, ch_itn);
i=1;

cur_tim=' ';


data_array = zeros(itn, colnum-1);

sObject=serial('COM6','BaudRate',115200,'TimeOut',10,'Terminator','LF');


get(sObject);
fopen(sObject);

fprintf(sObject,set1); 
sObject.ValuesSent %Confirms data being sent 


prev_t=0;
new_t=0;
tim=zeros(1,itn);
T=tic; %stopwatch starts 


while(i<((itn*52)*2)+1)
    
    fprintf(sObject,'*IDN?');
    scan = fscanf(sObject);
    
    tim(i)=toc(T);
    if i==1
       prev_t=0;
        
    else
       prev_t=tim(i-1);   
    end
    
    new_t=tim(i)-prev_t; 
    
    cur_tim=num2str(new_t,8);
      
    out=strcat(scan,',',cur_tim);
    data_array(i,:)=str2num(out); 
    prev_t=new_t;
    i=i+1;
    
end



fclose(sObject);
fprintf('Port Closed!\n'); 



end_tim=toc(T);
total_time=total_time+end_tim; 


data_array;
D1=[data_array itn_matrix];


pause(delay);



%% Header Printing Portion: Text File Header Setup

DateString = datestr(datetime);


file_data = fopen('SensorData.txt','w');
fprintf(file_data, 'Sensor Data Acquisition\n');
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, '%s\n\n', DateString);
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, '%s',test_name);
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, 'Distance to Target: %dm',distance);
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, 'Target Size: %s ', target_size);
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, 'Environment Information: Temp %dF, Humidity %d%% , Ambient Light %d lux', temperature, humidity, ambient_light);
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, 'Average Time Elapsed: %s sec', num2str((total_time)/(itn*7)));
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, 'Number of Samples Taken each Iteration: %d', itn);
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, 'Number of Samples Taken Total: %d samples',itn*7);
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, 'Total Time Elapsed: %s sec', num2str(total_time));
fclose(file_data);
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')

file_data = fopen('SensorData.txt','a');
fprintf(file_data, '%s',target_notes);
fclose(file_data);



%% Data Printing Section:
dlmwrite('SensorData.txt',space2,'delimiter',' ','newline', 'pc','-append')
dlmwrite('SensorData.txt',D1,'delimiter',',','newline', 'pc','-append')
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')
dlmwrite('SensorData.txt',space1,'delimiter',' ','newline', 'pc','-append')



%Confimation of data acquisition:
fprintf("Acquisition Complete\n")



