%   Загрузка логов и raw данных с IMU, перед загрузкой файлы должны быть
%   подготовлены. Необходимо убрать отрезок времени в начале, когда
%   комплекс еще не поймал сигнал GPS.
  
% [fileGPS,pathGPS] = uigetfile;
% dataLog = readmatrix([pathGPS,fileGPS]);
%  
% [fileIMU,pathIMU] = uigetfile;
% dataIMU = readmatrix([pathIMU,fileIMU]);  

%   Приведение raw и log данных к физическим величинам
normGyr =  (2 * pi / 360) * (250 / 32768);      % rad/s    
normMag = 1;                                    % mT
normAcc = 9.81 / 16384;                         % m/s^2
normVel = 10.0 / 36.0;                          % m/s

%   Калибровка магнитометра, полученная построением элипсоида и
%   преобразования его в сферу.
% MagBias     =   [-6.8135 9.5418 -12.2739];
% MagRotation =  [[ 0.9896 -0.0149  0.0013]
%                 [-0.0149  0.9682 -0.0036]
%                 [-0.0013 -0.0036  1.0440]];

%   Выбор частоты оцифровки IMU и GPS, Гц.
imuFs = 100;
gpsFs = 10;
imuSamplesPerGPS = (imuFs/gpsFs);
assert(imuSamplesPerGPS == fix(imuSamplesPerGPS), ...
    'GPS sampling rate must be an integer factor of IMU sampling rate.');

%   Вспомонательные константы
% N_gps = size(dataLog,1);
N_gps = round(size(dataIMU,1)/10);
N_start_raw = 1;

%   Заполенение массивов показаниями акселерометра и гироскопа.
dataAcc = dataIMU(N_start_raw:N_start_raw + N_gps * imuSamplesPerGPS - 1, 2:4);
dataGyr = dataIMU(N_start_raw:N_start_raw + N_gps * imuSamplesPerGPS - 1, 5:7);


% %   Синхронизация отсчетов по тикам и заполение массивов LLA и скорости
% dataGPS = zeros(N_gps,2);
% dataGPSvel = zeros(N_gps,2);
% dataMag = zeros(N_gps,3);

temp = zeros(N_gps,3);

for i = 1 : 1 : N_gps
    temp_val =  dataIMU(i*imuSamplesPerGPS,1);
%     [tmp, ind] = min(abs(dataLog(:,2) - temp_val));
    
%     dataGPS(i,:) = dataLog(ind, 8:9);
%     dataGPSvel(i,:) = dataLog(ind, 10:11);
    
    dataMag(i,:) = [mean(dataIMU((i-1)*imuSamplesPerGPS+1:i*imuSamplesPerGPS,8)) mean(dataIMU((i-1)*imuSamplesPerGPS+1:i*imuSamplesPerGPS,9)) mean(dataIMU((i-1)*imuSamplesPerGPS+1:i*imuSamplesPerGPS,10))];
    
%     temp(i,1) = dataLog(ind, 15);
%     temp(i,2) = dataLog(ind, 17);
%     temp(i,3) = dataLog(ind, 19);
end


%   Нормировка
dataAcc = dataAcc .* normAcc;
dataGyr = dataGyr .* normGyr;
dataMag = dataMag .* normMag;
% dataGPSvel(:,1) = dataGPSvel(:,1) * normVel;


%   Определяем стартовую координату для последующей симуляции.
% refloc = [dataGPS(1,1) dataGPS(1,2) 2];

%   Раскомментировать, если есть увереность, что при включении комплекс
%   находился в состоянии покоя на земле. Средние необходимы для
%   дальнейшего расчета.
meanVx = mean(dataGyr(1:100,1));
meanVy = mean(dataGyr(1:100,2));
meanVz = mean(dataGyr(1:100,3));
meanAx = mean(dataAcc(1:100,1));
meanAy = mean(dataAcc(1:100,2));
meanAz = mean(dataAcc(1:100,3))-9.81;

%   Получение референсного магнитного поля с помощью WMW
model_epoch = '2020';
decimal_year = 2020;
height = 2;
% [xyz, h, dec, dip, f] = wrldmagm(height, refloc(1), refloc(2), decimal_year, model_epoch);

% MagneticField = [-xyz(1) xyz(2) xyz(3)]./ 1000
% (dataMag(1,:) - MagBias) * MagRotation 
% mag = (dataMag(1,:) - MagBias) * MagRotation;

%   Инициализация объеета фильтра в системе NED 
fusionfilt1 = insfilterMARG('ReferenceFrame', 'NED');
fusionfilt1.IMUSampleRate = imuFs;
% fusionfilt1.ReferenceLocation = refloc;

% Инициализация вектора состояния системы 
initstate = zeros(22,1);
phi_init1 = deg2rad(temp(1,1));       %   THDG
% phi_init = deg2rad(dataGPSvel(1,2));    %    COG
% phi_init = 0;
initstate(1:4) = compact(quaternion(cos(phi_init1/2),0,0,sin(phi_init1/2)));    %   начальная ориентация
% initstate(1:4) = compact(quaternion(1,0,0,0));
initstate(5:7) = [0 0 1];                                                       %   положение в относительной система отсчета
% initstate(8:10) = [cos(phi_init) sin(phi_init) 0] * dataGPSvel(1,1);            %   начальная скорость
% initstate(8:10) = [0 0 0];        
initstate(11:13) =  [meanVy  meanVx -meanVz]./100 * 1.00;                       %   стартовое смещение нуля гироскопа
initstate(14:16) =  [-meanAy -meanAx meanAz]./100 * 1.00;                       %   стартовое смещение нуля аксерометра
% initstate(17:19) =  MagneticField;                                              %   магнитное поле в МСК
initstate(20:22) = [3 3 3];                                                     %   стартовое смещение нуля магнитометра

fusionfilt1.State = initstate;

%   Шум измерения с дастчиков 
Rmag = 0.05; % Magnetometer measurement noise
Rvel = 0.01; % GPS Velocity measurement noise
Rpos = 1.50; % GPS Position measurement noise

% Шум процесса, подбирается в ручную
fusionfilt1.AccelerometerBiasNoise =  2e-6;
fusionfilt1.AccelerometerNoise = 2e-4; 
fusionfilt1.GyroscopeBiasNoise = 1e-12;
fusionfilt1.GyroscopeNoise =  1e-10;
fusionfilt1.MagnetometerBiasNoise = 1e-12;
fusionfilt1.GeomagneticVectorNoise = 1e-10;

% Начальная матрица ковариации параметров
fusionfilt1.StateCovariance = 1e-9*ones(22);


usePoseView = true;  % Turn on the streaming error plot
if usePoseView
    posescope = HelperPoseViewer(...
        'XPositionLimits', [-150 150], ...
        'YPositionLimits', [-150, 150], ...
        'ZPositionLimits', [-15 15]);
end

% N_end = N_gps;
N_end = 5000;

pqorient = quaternion.zeros(N_end,1);
pqpos = zeros(N_end,3);
pmag = zeros(N_end,3);
resmag = zeros(N_end,3);

fcnt = 1;

% normRefMag = norm(MagneticField);
% deltaMag = abs(normRefMag * 0.2);

%   Цикл симулиции
for gps_step = 1:N_end
    %   Шаги без GPS и магнитометра
    for imu_step = 1:imuSamplesPerGPS
        ff = (gps_step - 1) * imuSamplesPerGPS + imu_step;
        
        accel1 = [-dataAcc(ff,2) -dataAcc(ff,1) dataAcc(ff,3)];
        gyro1 = [dataGyr(ff,2) dataGyr(ff,1) -dataGyr(ff,3)];
        
        predict(fusionfilt1, accel1, gyro1);
        [fusedPos, fusedOrient] = pose(fusionfilt1);
    end

%     %   Уточнение положения по gps
%     phi = deg2rad(dataGPSvel(gps_step,2));
%     vel_gps = [cos(phi) sin(phi) 0] * dataGPSvel(gps_step,1);
%     pose_gps = [dataGPS(gps_step,:) 1];
%     fusegps(fusionfilt1, pose_gps, Rpos, vel_gps, Rvel);
%     
%     
    %   Уточнение ориентации по магнитометру
    mag = (dataMag(gps_step,:) - MagBias) * MagRotation;
    magCor = [mag(2) mag(1) -mag(3)];
    
    %   Проверка на величину магнитной девиации
%     if abs(norm(magCor) - normRefMag) < deltaMag
%         [mag1,magCov] = fusemag(fusionfilt1, magCor, Rmag);
%     end 
    resmag(gps_step,:) = magCor;
%     
%     %   Лонирование положения и ориентации
%     pqorient(gps_step) = fusedOrient;
%     pqpos(gps_step,:) = fusedPos;
%     pmag(gps_step,:) = mag1;
    
    %   Раскомментировать для контроля времени расчета
%     fprintf("Time: %i min %i sec\n", idivide(int16(gps_step/gpsFs),int16(60)), mod(int16(gps_step/gpsFs),int16(60)));
    
    %   Раскомментировать для визуалиции расчета
    if usePoseView
        posescope(fusedPos, fusedOrient,  fusedPos, ...
        fusedOrient);
    end
    
end


%   Визуалиция результатов

eulerAnglesDegrees = eulerd(pqorient,'XYZ','frame');

for n =1:size(eulerAnglesDegrees(:,3))
    if eulerAnglesDegrees(n,3) < 0
        eulerAnglesDegrees(n,3) = 360 + eulerAnglesDegrees(n,3); 
    end
end

dy4 = zeros(N_end,3);
for i=3:(N_end-1)
 dy4(i,:)=(-pqpos(i+1,:)+27*pqpos(i,:)-27*pqpos(i-1,:)+pqpos(i-2,:))/(24);
end

pvel_phi = zeros(N_end,1);
for n = 3:N_end
    vel = dy4(n,:);
    if vel(2)>0
        phi = rad2deg( atan(vel(1)/vel(2)));
    else
        phi = rad2deg( atan(-vel(1)/-vel(2))) + 180;
    end
    if phi < 0
        pvel_phi(n-1) = 90-phi + 360;
    else
        pvel_phi(n-1) = 90-phi;
    end
end


figure(1)
tiledlayout(2,1)
nexttile
hold on
plot(unwrap(eulerAnglesDegrees(1:N_end,3)),'r')
plot(unwrap(temp(1:N_end,1)),'b')
ylabel('Angle, degree')
xlabel('Time, gpsFS * t')

% hold off
% 
% nexttile
% hold on
plot(rad2deg(unwrap(deg2rad(pvel_phi(1:N_end,1)))),'--r')
plot(rad2deg(unwrap(deg2rad(dataGPSvel(1:N_end,2)))),'--b')
lgd=legend('THDG-Kalman', 'THDG-GPS', 'COG-Kalman', 'COG-GPS');
lgd.Title.String = 'Method';
hold off

nexttile
hold on
plot(dataGPSvel(1:N_end,1),'b')
ylabel('Sog, m/s')
xlabel('Time, gpsFS * t')
hold off

% nexttile
% hold on
% plot(dataGyr(1:10:N_end*10,1),'r')
% plot(dataGyr(1:10:N_end*10,2),'b')
% plot(dataGyr(1:10:N_end*10,3),'g')
% ylabel('AngVel, deg/s')
% xlabel('Time, gpsFS * t')
% hold off

figure(2)
hold on
plot(resmag(1:N_end,1),'r')
plot(resmag(1:N_end,2),'b')
plot(-20*cos(deg2rad(temp(1:N_end,1))),'--r')
plot(20*sin(deg2rad(temp(1:N_end,1))),'--b')
plot(resmag(1:N_end,2),'b')
plot(resmag(1:N_end,3),'g')
ylabel('B, mT')
xlabel('Time, gpsFS * t')
hold off

figure(3)
tiledlayout(2,1)

nexttile
hold on
plot(eulerAnglesDegrees(1:N_end,2),'r')
plot(temp(1:N_end,2),'b')
ylabel('Roll, degree')
xlabel('Time, gpsFS * t')
lgd=legend('Kalman', 'Madgwick');
lgd.Title.String = 'Method';
hold off

nexttile
hold on
plot(eulerAnglesDegrees(1:N_end,1),'r')
plot(temp(1:N_end,3),'b')
ylabel('Heel, degree')
xlabel('Time, gpsFS * t')
lgd=legend('Kalman', 'Madgwick');
lgd.Title.String = 'Method';
hold off

figure(4)
hold on
plot(unwrap(eulerAnglesDegrees(1:N_end,3))-unwrap(temp(1:N_end,1)),'r')
ylabel('Angle, degree')
xlabel('Time, gpsFS * t')

plot(rad2deg(unwrap(deg2rad(pvel_phi(1:N_end,1))))-rad2deg(unwrap(deg2rad(dataGPSvel(1:N_end,2)))),'b')
lgd=legend('THDG_Kalman-THDG_GPS', 'COG_Kalman-COG_GPS');
lgd.Title.String = 'Method';
hold off