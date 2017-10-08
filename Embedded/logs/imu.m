%% 
subplot(3, 1, 1);
plot(time, gyro_x);
subplot(3, 1, 2);
plot(time, gyro_y);
subplot(3, 1, 3);
plot(time, gyro_z);

%% 
subplot(3, 1, 1);
plot(time, accel_x);
subplot(3, 1, 2);
plot(time, accel_y);
subplot(3, 1, 3);
plot(time, accel_z);

%% 
bias_gyro_x = mean(gyro_x(2:length(gyro_x)))
bias_gyro_y = mean(gyro_y(2:length(gyro_y)))
bias_gyro_z = mean(gyro_z(2:length(gyro_z)))

%% 
dt = [];
for n = [1:length(time)-1]
    dt(n) = time(n+1) - time(n);
end

plot(dt);

%% 
alpha = 0.3;
subplot(3, 1, 1);
plot(time, gyro_x); hold on;
plot(time, filter_exp(alpha, gyro_x)); hold off;
subplot(3, 1, 2);
plot(time, gyro_y); hold on;
plot(time, filter_exp(alpha, gyro_y)); hold off;
subplot(3, 1, 3);
plot(time, gyro_z); hold on;
plot(time, filter_exp(alpha, gyro_z)); hold off;

%% 
alpha = 0.98;
pitch = [];
yaw = [];
roll = [];

pitch(1) = 0;
yaw(1) = 0;
roll(1) = 0;

for n = 1:length(time)-1
    dt = (time(n+1) - time(n)) / 1000;
    pitch(n+1) = alpha*(pitch(n) + gyro_y(n)*dt) + (1-alpha)*(180/pi)*atan2(accel_x(n), sqrt(accel_z(n)^2 + accel_y(n)^2));
    roll(n+1) = alpha*(roll(n) + gyro_x(n)*dt) + (1-alpha)*(180/pi)*atan2(accel_y(n), sqrt(accel_z(n)^2 + accel_x(n)^2));
    yaw(n+1) = yaw(n) + gyro_z(n)*dt;
end

subplot(3, 1, 1);
plot(time, pitch);
subplot(3, 1, 2);
plot(time, yaw);
subplot(3, 1, 3);
plot(time, roll);
    