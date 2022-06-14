data_2 = load("demofile2.txt");
time = data_2(:,1);
x_l = data_2(:,2);
x_r = data_2(:,3);

scatter(time, x_r)
ylim([-12000 -4000])
xlim([0 20])
xlabel("time (s)")
ylabel("encoder pulses")

% Load data into variable  
raw_data = readmatrix("test.csv");

% Extract relevant data 
data = raw_data(:,5);

% Heading data 
heading = data(1 : 12 : end)/16; 

% Roll data 
roll = data(2 : 12 : end)/16; 

% Pitch data 
pitch = data(3 : 12 : end)/16; 

% Acceleration x 
accel_x = data(4: 12 : end)/100; 

% Acceleration y 
accel_y = data(5: 12 : end)/100; 

% Acceleration z 
accel_z = data(6: 12 : end)/100; 

% Magnetic Field x 
mag_x = data(7: 12 : end)/16; 

% Magnetic Field y 
mag_y = data(8: 12 : end)/16; 

% Magnetic Field z 
mag_z = data(9: 12 : end)/16; 

% Angular x 
ang_x = data(10: 12 : end)/16;

% Angular y 
ang_y = data(11: 12 : end)/16;

% Angular Z
ang_z = data(12:12:end)/16; 

% time data
imu_time = 0 : 1 : length(heading)-1; 
imu_time = transpose(imu_time);

% time data
imu_time_2 = 0 : 1 : length(heading)-2; 
imu_time_2 = transpose(imu_time_2);


% Plot 

figure()
plot(imu_time, heading)
xlabel("time (s)")
ylabel("heading (deg)")
title("heading")

figure()
plot(imu_time, pitch)
xlabel("time (s)")
ylabel("pitch (deg)")
title("pitch")

figure()
plot(imu_time, roll)
xlabel("time (s)")
ylabel("roll(deg)")
title("roll")


figure()
plot(imu_time, accel_x)
xlabel("time (s)")
ylabel("acceleration x (m/s^2)")
title("acceleration x")

figure()
plot(imu_time, accel_y)
xlabel("time (s)")
ylabel("acceleration y (m/s^2)")
title("acceleration y")


figure()
plot(imu_time, accel_z)
xlabel("time (s)")
ylabel("acceleration z (m/s^2)")
title("acceleration z")


figure()
plot(imu_time, mag_x)
xlabel("time (s)")
ylabel("magnetic field x (uT)")
title("Magnetic field x")

figure()
plot(imu_time, mag_y)
xlabel("time (s)")
ylabel("magnetic field y (uT)")
title("Magnetic field y")

figure()
plot(imu_time, mag_z)
xlabel("time (s)")
ylabel("magnetic field z (uT)")
title("Magnetic field z")

figure()
plot(imu_time_2, ang_x)
xlabel("time (s)")
ylabel("angular velocity x")
title("angular velocity")

figure()
plot(imu_time_2, ang_y)
xlabel("time (s)")
ylabel("angular velocity y")
title("angular velocity")

figure()
plot(imu_time_2, ang_z)
xlabel("time (s)")
ylabel("angular velocity z")
title("angular velocity")



