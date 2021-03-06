clc
close all
clear all
uigetfile = "01-01-00_00_00_01.csv";
%uigetfile = "COACH_1-01-2000_00_00_00_rawBMX160.csv";
fileMain = uigetfile;
dataMain = readmatrix(fileMain); 
[sz, var9] = size(dataMain);
uigetfile = "01-01-00_00_00_01_bmx.csv";
%uigetfile = "COACH_1-01-2000_00_00_00_rawBMX160.csv";
fileIMU = uigetfile;
dataIMU = readmatrix(fileIMU );  

[imu_sz, var4] = size(dataIMU);
%%
start_pos = 24000;
stop = size(dataMain);
%stop_pos = stop(1);
stop_pos = 26000;
%%
% %   ?????????? raw ? log ?????? ? ?????????? ?????????
% normGyr =  (2 * pi / 360) * (250 / 32768);      % rad/s    
% normMag = 1;                                    % mT
% normAcc = 9.81 / 16384;                         % m/s^2
% normVel = 10.0 / 36.0;                          % m/s
% 
% 
% magn = dataIMU(:,8:10);
% %magnetometer data, simply call the function as: 
% [A,b,expMFS]  = magcal(magn);
% xCorrected = (magn-b)*A; 
% 
% 
% % Plot the original and corrected data. Show the ellipsoid that best fits
% % the original data. Show the sphere on which the corrected data should
% % lie.
% 
% 
% 
% de = HelperDrawEllipsoid;
% de.plotCalibrated(A,b,expMFS,magn,xCorrected,'sym');
% 
% iter = 1;
% 
% imu_start_pos = 1;
% imu_stop_pos = 1;
% 
% for iter = 1:imu_sz
%     if dataIMU(iter,1) > dataMain(start_pos,1)
%         imu_start_pos = iter;
%         break
%     end
% end
% 
% 
% for iter = imu_start_pos:imu_sz
%     if dataIMU(iter,1) > dataMain(stop_pos,1)
%         imu_stop_pos = iter;
%         break
%     end
% end
% 
% imu_start_pos
% imu_stop_pos
% 
% 
% magn_calib = magn - b;
% 
% course_calc = zeros(stop_pos,1);
% 
% arr = dataIMU(:,8:10);
% acel = dataIMU(:,2:4);
% gyro = dataIMU(:,5:7);
% 
% acc = acel .* normAcc;
% mag = magn_calib .* normMag;
% gyro = gyro .* normGyr;
% 
% orientation = zeros(imu_sz,4);
% 
% start = 1;
% stop = imu_sz;
% 
% ifilt = ahrsfilter('SampleRate', 100);
% for ii=imu_start_pos:imu_stop_pos
%     qahrs = ifilt(acc(ii,:), gyro(ii,:), mag(ii,:));
%     %viewer(qahrs);
%     orientation(ii,1) = dataIMU(ii,1);
%     eul = quat2eul(qahrs); 
%     orientation(ii,2) = eul(1);
%     orientation(ii,3) = eul(2);
%     orientation(ii,4) = eul(3);
%     %pause(0.001);
% end
% 
% for i = start_pos:stop_pos
%    for j = imu_start_pos:imu_stop_pos
%       if dataIMU(j,1) > dataMain(i,1)
%        course_calc(i) = rad2deg(orientation(j,2));
%        break
%       end
%    end
%     
% end
% 
% figure
% plot (orientation(imu_start_pos:imu_stop_pos,1), orientation(imu_start_pos:imu_stop_pos,2))
% 

%%




dist = zeros(stop_pos,1);
course = zeros(stop_pos,1);
speed_km_h = zeros(stop_pos,1);
speed_knots = zeros(stop_pos,1);
hdg = dataMain(:,6);


lat = dataMain(:,9);
lon = dataMain(:,10);

rudder_pos = dataMain(:,15);
%rudder_angl = (rudder_pos - 344) * 2;
rudder_angl = normalize_var(rudder_pos, -35, 35);
cog = dataMain(:,11);
sog = dataMain(:,12) * 0.0036;
geoplot (lat(start_pos:stop_pos), lon(start_pos:stop_pos));

for i = start_pos+1 : stop_pos
    dist(i) = deg2km(distance(lat(i-1), lon(i-1), lat(i), lon(i)));
    speed_km_h(i) = dist(i) / 1 * 3600;
    speed_knots(i) = speed_km_h(i) / 1.854;
    course(i) = getCourceToWaypoint(lat(i), lon(i), lat(i-10), lon(i-10)) - 180;
end


figure
subplot(2,1,1)
plot(dataMain(start_pos:stop_pos,1), speed_km_h(start_pos:stop_pos),...
    dataMain(start_pos:stop_pos,1), sog(start_pos:stop_pos))
ylabel('????????, ?? ? ???')
xlabel('?????, ??')
title('????????, ?? ? ???')


subplot(2,1,2)
plot(dataMain(start_pos:stop_pos,1), speed_knots(start_pos:stop_pos))

title('????????, ????')
ylabel('????????, ????')
xlabel('?????, ??')

%course_calc = course_calc + 60;
hdg = hdg + 130;
windowSize = 20; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
rudder_angl = filter(b,a,rudder_angl);
figure 
plot(dataMain(start_pos:stop_pos,1), cog(start_pos:stop_pos),...
    dataMain(start_pos:stop_pos,1), hdg(start_pos:stop_pos),...
    dataMain(start_pos:stop_pos,1), rudder_angl(start_pos:stop_pos))
     
    
%dataMain(start_pos:stop_pos,1), course_calc(start_pos:stop_pos),...
title('????????? ????? ? ???? ???? ????, ????')
ylabel('???? ? ????, ????')
xlabel('?????, ??')
legend("COG", "HDG", "???? ???? ????")


% figure 
% plot(dataMain(start_pos:stop_pos,1), dataMain(start_pos:stop_pos,15),...
%     dataMain(start_pos:stop_pos,1), dataMain(start_pos:stop_pos,16))