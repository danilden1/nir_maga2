

%% Ideal Magnetometers
% An ideal three-axis magnetometer measures magnetic field strength along
% orthogonal X, Y and Z axes. Absent any magnetic interference,
% magnetometer readings measure the Earth's magnetic field. If
% magnetometer measurements are taken as the sensor is rotated through all
% possible orientations, the measurements should lie on a sphere. The
% radius of the sphere is the magnetic field strength.

% 
clc
close all
clear all

uigetfile = "01-01-00_00_00_01_bmx.csv";
%uigetfile = "COACH_1-01-2000_00_00_00_rawBMX160.csv";
fileIMU = uigetfile;
dataIMU = readmatrix(fileIMU );  
%%
%   Приведение raw и log данных к физическим величинам
normGyr =  (2 * pi / 360) * (250 / 32768);      % rad/s    
normMag = 1;                                    % mT
normAcc = 9.81 / 16384;                         % m/s^2
normVel = 10.0 / 36.0;                          % m/s


%%

coord = dataIMU(:,1:10);
last_time = coord(1,1);
last_coord = coord(1,:);
[sz,var] = size(coord(:,1));
count = 1;
int16_max = 32767;
filt_trace_acel = int16_max; %порог фильтрации
filt_trace_mag = 200; %порог фильтрации
filt_trace_gyro = 10000; %порог фильтрации
filt_trace_mag_2 = 20; %порог фильтрации по производной
filt_trace_acel_2 = 10000; %порог фильтрации по производной
filt_trace_gyro_2 = 10000; %порог фильтрации по производной
% while count <= sz
%     skip_flag = 0;
%     if count == 2637
%         var = 2;
%     end
%     for i = 2:10
%         if isnan(coord(count,i))
%             skip_flag = skip_flag + 1;
%         end
%     end
%     for i = 2:4
%         if abs(coord(count,i)) > filt_trace_acel
%             skip_flag = skip_flag + 1;
%         end
%         if abs(coord(count,i)- last_coord(i)) > filt_trace_acel_2
%             skip_flag = skip_flag + 1;
%         end
%     end
%     for i = 5:7
%         if abs(coord(count,i)) > filt_trace_gyro
%             skip_flag = skip_flag + 1;
%         end
%         if abs(coord(count,i)- last_coord(i)) > filt_trace_gyro_2
%             skip_flag = skip_flag + 1;
%         end
%     end
%     for i = 8:10
%         if abs(coord(count,i)) > filt_trace_mag
%             skip_flag = skip_flag + 1;
%         end
%         if abs(coord(count,i)- last_coord(i)) > filt_trace_mag_2
%             skip_flag = skip_flag + 1;
%         end
%     end
%     
%     if coord(count,1) < last_time
%         skip_flag = skip_flag + 1;
%     end
%     if skip_flag
%         coord(count,:) = [];
%         sz = sz - 1;
%         last_coord = coord(count,:);
%         last_time = coord(count,1);
%     else
%         last_coord = coord(count,:);
%         last_time = coord(count,1);
%         count = count + 1;
%     end
% 
% 
% 
% end
start_t = 1;
[stop_t, var2] = size(coord(:,1));
%stop_t = 100;
arr = coord(:,8:10);
acel = coord(:,2:4);
gyro = coord(:,5:7);
magn = arr;

figure
axis equal
scatter3(arr(:,1),arr(:,2),arr(:,3));
title('Ideal Magnetometer Data');
%%
%magnetometer data, simply call the function as: 
[A,b,expMFS]  = magcal(magn);
xCorrected = (magn-b)*A; 


% Plot the original and corrected data. Show the ellipsoid that best fits
% the original data. Show the sphere on which the corrected data should
% lie.

de = HelperDrawEllipsoid;
de.plotCalibrated(A,b,expMFS,magn,xCorrected,'sym');


%%

figure
subplot(3,1,1)
plot(coord(start_t:stop_t,1), coord(start_t:stop_t,2), coord(start_t:stop_t,1), ...
coord(start_t:stop_t,3),  coord(start_t:stop_t,1), coord(start_t:stop_t,4))
title('Axel')
legend('x', 'y', 'z')
%ylabel('deg')
xlabel('ticks, ms')
subplot(3,1,2)
plot(coord(start_t:stop_t,1), coord(start_t:stop_t,5), coord(start_t:stop_t,1), ...
coord(start_t:stop_t,6),  coord(start_t:stop_t,1), coord(start_t:stop_t,7))
title('Gyro')
legend('x', 'y', 'z')
%ylabel('deg')
xlabel('ticks, ms')
subplot(3,1,3)
plot(coord(start_t:stop_t,1), coord(start_t:stop_t,8), coord(start_t:stop_t,1), ...
coord(start_t:stop_t,9),  coord(start_t:stop_t,1), coord(start_t:stop_t,10))
title('Magn')
legend('x', 'y', 'z')
%ylabel('deg')
xlabel('ticks, ms')
A
b

zero_mat = eye(3);
zero_mat = int32(zero_mat * 10000);

s = "zeromat"
st = "FusionMatrixGyr="+zero_mat(1,1)+","+zero_mat(1,2)+","+zero_mat(1,3)...
                        +","+zero_mat(2,1)+","+zero_mat(2,2)+","+zero_mat(2,3)...
                        +","+zero_mat(3,1)+","+zero_mat(3,2)+","+zero_mat(3,3)
 
st = "FusionMatrixAcc="+zero_mat(1,1)+","+zero_mat(1,2)+","+zero_mat(1,3)...
                        +","+zero_mat(2,1)+","+zero_mat(2,2)+","+zero_mat(2,3)...
                        +","+zero_mat(3,1)+","+zero_mat(3,2)+","+zero_mat(3,3)
                    
FUSION_ROTATION_MATRIX_IDENTITY_COACH = A;
out = FUSION_ROTATION_MATRIX_IDENTITY_COACH;  
out = int32(out * 10000);

scm = "coach"
str = "FusionMatrixMag="+out(1,1)+","+out(1,2)+","+out(1,3)...
                        +","+out(2,1)+","+out(2,2)+","+out(2,3)...
                        +","+out(3,1)+","+out(3,2)+","+out(3,3)
outh1 = b;
outh1 = int32(outh1 * 10000);

                    
str11 = "HardIronBias=" + outh1(1) + "," + outh1(2) + "," + outh1(3)


% ma=zeros(3,2);
% ma(1,1) = max(magn(:,1));
% ma(1,2) = min(magn(:,1));
% ma(2,1) = max(magn(:,2));
% ma(2,2) = min(magn(:,2));
% ma(3,1) = max(magn(:,3));
% ma(3,2) = min(magn(:,3));
% ma
% ma(1,1) = ma(1,1) - b(1);
% ma(1,2) = ma(1,2) - b(1);
% ma(2,1) = ma(2,1) - b(2);
% ma(2,2) = ma(2,2) - b(2);
% ma(3,1) = ma(3,1) - b(3);
% ma(3,2) = ma(3,2) - b(3);
% 
% ma

%%
magn_calib = magn - b;
%magn_calib = magn;

viewer = HelperOrientationViewer;


acc = acel .* normAcc;
mag = magn_calib .* normMag;
gyro = gyro .* normGyr;

orientation = zeros(sz,4);

start = 1;
stop = sz;

ifilt = ahrsfilter('SampleRate', 100);
for ii=start:stop
    qahrs = ifilt(acc(ii,:), gyro(ii,:), mag(ii,:));
    %viewer(qahrs);
    orientation(ii,1) = dataIMU(ii,1);
    eul = quat2eul(qahrs); 
    orientation(ii,2) = eul(1);
    orientation(ii,3) = eul(2);
    orientation(ii,4) = eul(3);
    %pause(0.001);
end

figure
plot (orientation(start:stop,1), orientation(start:stop,2),...
        orientation(start:stop,1), orientation(start:stop,3),...
        orientation(start:stop,1), orientation(start:stop,4))

