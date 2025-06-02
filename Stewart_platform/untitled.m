%% simulated_stewart_platform.m
% Stewart platform simulation without Arduino hardware

clc;
clear;
close all;

%% --- Parameters (same as original) ---
moveX = 1;
moveY = 1;
moveZ = 1;
DegY = 1;
DegZ = 1;

l_connecting_rod = 38.65;      %[mm]
r_sta_pivot = 61.71;           %[mm] Radius to stationary pivot points
d_horn_motor = 14.57;         %[mm] diameter of rotary motor
Horn_connecting_rod_distance = 3.85;
offset_angle = 0;

p3_theta_z = deg2rad([330 30 90 150 210 270]');
p1_theta_z = deg2rad([353.6875 6.3125 113.6875 126.3125 233.6875 246.3125]');
p1_x = r_sta_pivot*cos(p1_theta_z);
p1_y = r_sta_pivot*sin(p1_theta_z);
p1_z = zeros(6,1);

degreeX = 0;
degreeY = 0;
degreeZ = 0;

r_dyn_pivot = 59.51;           %[mm] Radius to dynamic pivot points
z_dynamic_home = 32.525;
x_translation = 0;             %[mm]
y_translation = 0;             %[mm]
z_translation = 10;            %[mm]

deviceNeutral = 512;           % Simulated analog mid-value
maxIter = 500;                 % Number of simulation steps

%% --- Create figure handles ---
fig1 = figure('Name','3D Stewart Platform','NumberTitle','off');
fig2 = figure('Name','Leg Lengths','NumberTitle','off');

%% --- Simulation loop (finite) ---
for iter = 1:maxIter
    % Exit if user closed the figure
    if ~ishandle(fig1)
        break;
    end

    % --- Simulate analogRead inputs as neutral (no manual change) ---
    val1 = deviceNeutral;
    val2 = deviceNeutral;
    val3 = deviceNeutral;
    val4 = deviceNeutral;
    val5 = deviceNeutral;

    % --- Auto-motion logic ---
    z_translation = z_translation + moveZ;
    if z_translation >= 10
        moveZ = -1;
    elseif z_translation <= -10
        moveZ = 1;
    end
    x_translation = x_translation + moveX;
    if x_translation >= 5
        moveX = -1;
    elseif x_translation <= -5
        moveX = 1;
    end
    degreeY = degreeY + DegY;
    if degreeY >= 10
        DegY = -1;
    elseif degreeY <= -10
        DegY = 1;
    end
    degreeZ = degreeZ + DegZ;
    if degreeZ >= 10
        DegZ = -1;
    elseif degreeZ <= -10
        DegZ = 1;
    end

    % --- Update rotations ---
    x_rotation = deg2rad(degreeX);
    y_rotation = deg2rad(degreeY);
    z_rotation = deg2rad(degreeZ);

    % --- Calculate dynamic pivot coordinates ---
    p3_x = r_dyn_pivot*cos(p3_theta_z).*cos(x_rotation).*cos(z_rotation) + ...
           r_dyn_pivot*sin(p3_theta_z).*(sin(x_rotation).*sin(y_rotation).*cos(z_rotation) - cos(x_rotation).*sin(z_rotation)) + x_translation;
    p3_y = r_dyn_pivot*cos(p3_theta_z).*cos(y_rotation).*sin(z_rotation) + ...
           r_dyn_pivot*sin(p3_theta_z).*(sin(x_rotation).*sin(y_rotation).*sin(z_rotation) + cos(x_rotation).*cos(z_rotation)) + y_translation;
    p3_z = -r_dyn_pivot*cos(p3_theta_z).*sin(y_rotation) + ...
           r_dyn_pivot*sin(p3_theta_z).*sin(x_rotation).*cos(z_rotation) + z_dynamic_home + z_translation;

    % --- Compute leg lengths and servo angles ---
    for i = 1:6
        lengths(i) = sqrt((p1_x(i)-p3_x(i))^2 + (p1_y(i)-p3_y(i))^2 + (p1_z(i)-p3_z(i))^2);
        flat_length = sqrt(l_connecting_rod^2 - Horn_connecting_rod_distance^2);
        ServoPos(i) = floor(acos(((d_horn_motor^2) - lengths(i)^2 + flat_length^2) / (lengths(i)*d_horn_motor*2)) * (180/pi) + offset_angle);
    end
    simServoAngles = ServoPos; % for inspection

    % --- Plotting 3D configuration ---
    figure(fig1);
    clf;
    plot3(p3_x, p3_y, p3_z, 'r-o', p1_x, p1_y, p1_z, '-o');
    axis([-100 100 -100 100 -25 100]);
    hold on;
    for i = 1:6
        [Lx, Ly, Lz] = threedim_line_coords2(i, p1_x, p1_y, p1_z, p3_x, p3_y, p3_z);
        plot3(Lx, Ly, Lz, 'g-');
    end
    hold off;
    title(sprintf('Step %d / %d', iter, maxIter));
    drawnow;

    % --- Bar plot of leg lengths ---
   % figure(fig2);
  %  clf;
   % bar(lengths);
    % axis([0.5 6.5 0 100]);
    % title('Leg Lengths');
    % drawnow;
end

%% Local function
function [A, B, C] = threedim_line_coords2(i, p1_x, p1_y, p1_z, p3_x, p3_y, p3_z)
    A = [p1_x(i), p3_x(i)];
    B = [p1_y(i), p3_y(i)];
    C = [p1_z(i), p3_z(i)];
end
