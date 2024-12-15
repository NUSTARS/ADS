%code to filter subscale data to get euler angles bc original gyro data is
%rad/s

data = readtable('SFT1_primary.csv');

% subscale data gives accel, gyro data (rad/s), magnometer data
accelData = [data.accel_x, data.accel_y, data.accel_z];
gyroData = [data.gyro_roll, data.gyro_pitch, data.gyro_yaw];
magData = [data.mag_x, data.mag_y, data.mag_z];
altimeterData = data.height;  
timeData = data.time;

% only care about end of boost -> drouge (coast) 18.38
timeIdx = (timeData >= 2.98) & (timeData <= 12);
timeDataFiltered = timeData(timeIdx);
accelDataFiltered = accelData(timeIdx, :);
gyroDataFiltered = gyroData(timeIdx, :);
magDataFiltered = magData(timeIdx, :);
altimeterDataFiltered = altimeterData(timeIdx);

N = sum(timeIdx);
actQ = zeros(N, 4); % Quaternion orientation to save data
height = zeros(N, 1); 

% Initialize AHRS fusion filter
imuFs = 100; % IMU sample rate
fusionfilt = ahrs10filter('IMUSampleRate', imuFs);

% Configure AHRS filter parameters 

%based on the BMI088 sensor 
% https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf 
fusionfilt.AccelerometerNoise = [230e-6, 230e-6, 230e-6];
fusionfilt.GyroscopeNoise = deg2rad([0.014, 0.014, 0.014]);
fusionfilt.GeomagneticVectorNoise = 50e-6;
magNoise = 0.05;

%For each time step
for ii = 1:N
    accel = accelDataFiltered(ii, :);
    gyro = deg2rad(gyroDataFiltered(ii, :));
    mag = magDataFiltered(ii, :);
    alt = altimeterDataFiltered(ii);

    % Predict orientation using accelerometer and gyroscope
    predict(fusionfilt, accel, gyro);

    % Fuse magnetometer data
    fusemag(fusionfilt, mag, magNoise);

    % Log the estimated pose
    actQ(ii, :) = fusionfilt.State(1:4); % Quaternion orientation
    height(ii) = alt; 
end

eulerAngles = quat2eul(actQ, 'ZYX'); % Convert to Roll, Pitch, Yaw
eulerAnglesDeg = rad2deg(eulerAngles); % Convert to degrees for plotting

% % Plot results
% figure;
% subplot(3, 1, 1);
% plot(timeDataFiltered, eulerAnglesDeg(:, 1)); 
% title('Yaw (degrees)');
% xlabel('Time (s)');
% ylabel('Yaw');
% 
% subplot(3, 1, 2);
% plot(timeDataFiltered, eulerAnglesDeg(:, 2));  
% title('Pitch (degrees)');
% xlabel('Time (s)');
% ylabel('Pitch');
% 
% subplot(3, 1, 3);
% plot(timeDataFiltered, eulerAnglesDeg(:, 3));  
% title('Roll (degrees)');
% xlabel('Time (s)');
% ylabel('Roll');
% 
% % Plot height results
% figure;
% plot(timeDataFiltered, height * 3.28084); 
% title('Estimated Height (feet)');
% xlabel('Time (s)');
% ylabel('Height (feet)');

%Initialize angular positions for roll, pitch, and yaw
angularPosition = zeros(size(gyroDataFiltered)); % Nx3 matrix

% Time differences (delta t)
dt = diff(timeDataFiltered); % (N-1)x1 vector

% Numerical integration to calculate angular positions
for i = 2:length(timeDataFiltered)
    angularPosition(i, :) = angularPosition(i-1, :) + gyroDataFiltered(i-1, :) .* dt(i-1);
end

% Plotting: Overlay with the fusion filter's roll, pitch, yaw results
figure;

% Roll
subplot(3, 1, 1);
plot(timeDataFiltered, eulerAnglesDeg(:, 3), 'r', 'DisplayName', 'Fusion Filter Roll');
hold on;
plot(timeDataFiltered, angularPosition(:, 1), 'b--', 'DisplayName', 'Integrated Roll');
hold off;
title('Roll Comparison');
xlabel('Time (s)');
ylabel('Degrees');
legend('Location', 'Best');

% Pitch
subplot(3, 1, 2);
plot(timeDataFiltered, eulerAnglesDeg(:, 2), 'r', 'DisplayName', 'Fusion Filter Pitch');
hold on;
plot(timeDataFiltered, angularPosition(:, 2), 'b--', 'DisplayName', 'Integrated Pitch');
hold off;
title('Pitch Comparison');
xlabel('Time (s)');
ylabel('Degrees');
legend('Location', 'Best');

% Yaw
subplot(3, 1, 3);
plot(timeDataFiltered, eulerAnglesDeg(:, 1), 'r', 'DisplayName', 'Fusion Filter Yaw');
hold on;
plot(timeDataFiltered, angularPosition(:, 3), 'b--', 'DisplayName', 'Integrated Yaw');
hold off;
title('Yaw Comparison');
xlabel('Time (s)');
ylabel('Degrees');
legend('Location', 'Best');

% Adjust layout
sgtitle('Comparison of Fusion Filter vs Integrated Angular Positions');