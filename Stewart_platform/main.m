%%

close; clc; clear;


%% Set Constants

base_diameter = 18; %[inches], acrylic platform bolt circle
top_diameter = 14; %[inches], acrylic platform bolt circle
a = (base_diameter)/2; %[inches], distance from center of base to actuator
b = (top_diameter)/2; %[inches], distance from center of top to actuator

%Set base and top angles
base_angle = [ 0 60 120 180 240 300 ]; %[degrees], Spacing of actuator pairs, base
base_angle = sort(base_angle); %sorted for calculation integrity
%sort 는 배열의 원소를 오름차순 반환

top_angle = [ 0 40 120 160 240 280 ]; %[degrees], Spacing of actuator pairs, top
top_angle = sort(top_angle); %sorted for calculation integrity

% Set configuration
% 6-3 Configuration when the base joints are spaced 60° and the top joints
% pairs are spaced ~120° (Physical joint angle accounted for)
% 6-6 Configuration when all joints are spaced 60° (unstable configuration)
configuration_6_3 = false; % Run preset 6-3 configuration simulation
configuration_6_6 = false; % Run preset 6-6 configuration simulation
animation = true; % Display Animation
plots = true; % Display Plots

%Set physical parameters
stroke_length = 8; %[inches]
joint_angle = 3.47; %[degrees], for 6_3 configuration
min_joint_length = 16.7; %[inches], from CAD
max_joint_length = 24.7; %[inches], from CAD
min_joint_velocity = -2; %[inches/sec], from Progressive Automations후퇴속도
max_joint_velocity = 2; %[inches/sec], from Progressive Automations연장속도

% Set up motion variables
n = 1:400; % Array dimension
[theta,phi,psi,Px,Py,Pz]=deal(zeros(1,length(n))); % Create position array
time_duration= 100; % Seconds
time = 0:(time_duration/(max(n)-1)):time_duration; % Create time array, position
time_dot = time; % Create time array, velocity
time_dot(end)=[]; % Delete array end, discrete differentiation
motion_profile = 2; % Choose motion profile

% Preallocation, create joint locations and distance arrays
a_i = zeros(4,6,length(theta));
b_i = zeros(4,6,length(theta));
b_rot = zeros(4,6,length(theta));
L_vector = zeros(4,6,length(theta));
L_length = zeros(length(theta),6);
L_length_dot = L_length;
L_length_dot(length(n),:)=[];

%% Motion Profile Creation
if motion_profile == 1
    for m = 1:length(n)
        theta(m) = -30 + 30 * m/max(n); %[degrees], relative to x axis
        phi(m) = 0; %[degrees], relative to y axis
        psi(m) = -30 + 60 * m/max(n); %[degrees], relative to z axis
        Px(m) = 0; %[inches], x position
        Py(m) = 0; %[inches], y position
        Pz(m) = 20 + 2*cosd(m/max(n)*360); %[inches], z position
    end
end

if motion_profile == 2
    for m = 1:length(n)
        theta(m) = 10; %[degrees], relative to x axis
        phi(m) = 10*cosd(m/max(n)*360); %[degrees], relative to y axis
        psi(m) = 0; %[degrees], relative to z axis
        Px(m) = 3 * m/max(n); %[inches], x position
        Py(m) = 0; %[inches], y position
        Pz(m) = 22 - 3*m/max(n) ; %[inches], z position
    end
end

if motion_profile == 3
    for m = 1:length(n)
    theta(m) = 0; %[degrees], relative to x axis
    phi(m) = 10; %[degrees], relative to y axis
    psi(m) = -15 + 25 * m/max(n); %[degrees], relative to z axis
    Px(m) = 3; %[inches], x position
    Py(m) = -2 + 3 * m/max(n); %[inches], y position
    Pz(m) = 20; %[inches], z position
    end
end

if motion_profile == 4
    for m = 1:length(n)
        theta(m) = -15 + 30 * m/max(n); %[degrees], relative to x axis
        phi(m) = 10 - 5 * m/max(n); %[degrees], relative to y axis
        psi(m) = 15*sind(m/max(n)*360); %[degrees], relative to z axis
        Px(m) = 1.5; %[inches], x position
        Py(m) = -3 + 4*m/max(n); %[inches], y position
        Pz(m) = 19.8 + 2*cosd(m/max(n)*360);%[inches], z position
    end
end

P = [Px;Py;Pz;zeros(1,length(theta))]; %combine into array

%% Calculate EOMs

if configuration_6_6
    for j = 1:length(theta)
        for i = 1:6
            %calculate a vector for each base location
            a_i(1,i,j)=a*cosd((i-1)*60);
            a_i(2,i,j)=a*sind((i-1)*60);
            a_i(3,i,j)=0;
            a_i(4,i,j)=1;
            %calculate b vector for each top location
            b_i(1,i,j)=b*cosd((i-1)*60);
            b_i(2,i,j)=b*sind((i-1)*60);
            b_i(3,i,j)=0;
            b_i(4,i,j)=1;
            %convert b from global to rotated
            b_rot(:,i,j)=stewartrot(theta(j),phi(j),psi(j),b_i(:,i,j));
            L_vector(:,i,j) = P(:,j) + b_rot(:,i,j) - a_i(:,i,j);
            L_length(j,i) = sqrt(L_vector(1,i,j)^2 + L_vector(2,i,j)^2 + L_vector(3,i,j)^2);
        end
    end
elseif configuration_6_3
    for j = 1:length(theta)
        %calculate b vector for each top location
        for i = [1 3 5]
            b_i(1,i,j)=b*cosd((i-1)*60-joint_angle);
            b_i(2,i,j)=b*sind((i-1)*60-joint_angle);
            b_i(3,i,j)=0;
            b_i(4,i,j)=1;
            %------------------------------------------
            b_i(1,i+1,j)=b*cosd((i-1)*60+joint_angle);
            b_i(2,i+1,j)=b*sind((i-1)*60+joint_angle);
            b_i(3,i+1,j)=0;
            b_i(4,i+1,j)=1;
        end
        for i = 1:6
            %calculate a vector for each base location
            a_i(1,i,j)=a*cosd((i-1)*60);
            a_i(2,i,j)=a*sind((i-1)*60);
            a_i(3,i,j)=0;
            a_i(4,i,j)=1;
            %convert b from global to rotated
            b_rot(:,i,j)=stewartrot(theta(j),phi(j),psi(j),b_i(:,i,j));
            L_vector(:,i,j) = P(:,j) + b_rot(:,i,j) - a_i(:,i,j);
            L_length(j,i) = sqrt(L_vector(1,i,j)^2 + L_vector(2,i,j)^2 + L_vector(3,i,j)^2);
        end
    end
else %for all non-true 6-6 and 6-3 cases
    for j = 1:length(theta)
        for i = 1:6
            a_i(1,i,j)=a*cosd(base_angle(i));
            a_i(2,i,j)=a*sind(base_angle(i));
            a_i(3,i,j)=0;
            a_i(4,i,j)=1;
            %------------------------------------------
            b_i(1,i,j)=b*cosd(top_angle(i));
            b_i(2,i,j)=b*sind(top_angle(i));
            b_i(3,i,j)=0;
            b_i(4,i,j)=1;
            b_rot(:,i,j)=stewartrot(theta(j),phi(j),psi(j),b_i(:,i,j));
            L_vector(:,i,j) = P(:,j) + b_rot(:,i,j) - a_i(:,i,j);
            L_length(j,i) = sqrt(L_vector(1,i,j)^2 + L_vector(2,i,j)^2 + L_vector(3,i,j)^2);
        end
    end
end

%% Derivative Calculation

[theta_dot,phi_dot,psi_dot,Px_dot,Py_dot,Pz_dot,P_dot]=deal(zeros(1,length(n)-1)); %preallocation

for i=2:length(n)
    Px_dot(i-1) = (Px(i) - Px(i-1))/(time(i)-time(i-1));
    Py_dot(i-1) = (Py(i) - Py(i-1))/(time(i)-time(i-1));
    Pz_dot(i-1) = (Pz(i) - Pz(i-1))/(time(i)-time(i-1));
    P_dot(i-1) = sqrt((Px_dot(i-1))^2 + (Py_dot(i-1))^2 + (Pz_dot(i-1))^2);
    theta_dot(i-1) = (theta(i) - theta(i-1))/(time(i)-time(i-1));
    phi_dot(i-1) =(phi(i) - phi(i-1))/(time(i)-time(i-1));
    psi_dot(i-1) =(psi(i) - psi(i-1))/(time(i)-time(i-1));
    for j = 1:6
        L_length_dot(i-1,j) = (L_length(i,j) - L_length(i-1,j))/(time(i)-time(i-1));
    end
end

%% Plots
if plots
    figure(1)
    hold on
    for i=1:6
        plot(time,L_length(:,i))
    end
    xlabel('Time (seconds)')
    ylabel('Length (inches)')
    plot(time, min_joint_length*ones(1,length(time)),'--k', time, max_joint_length*ones(1,length(time)),'--k','LineWidth',2)
    legend('Actuator 1', 'Actuator 2', 'Actuator 3', 'Actuator 4', 'Actuator 5', 'Actuator 6', 'Min Length', 'Max Length', 'Location', 'north')
    title('Joint Lengths, from Sphere to Sphere')
    hold off

    figure(2)
    hold on
    for i=1:6
    plot(time_dot,L_length_dot(:,i))
    end
    plot(time, min_joint_velocity*ones(1,length(time)),'--k', time,    max_joint_velocity*ones(1,length(time)),'--k', 'LineWidth',2)
    xlabel('Time (seconds)')
    ylabel('Velocity (inches/sec)')
    ylim([-2.5 2.5])
    % plot(time, min_joint_length*ones(1,length(time_dot)), time, max_joint_length*ones(1,length(time)))
    legend('Actuator 1', 'Actuator 2', 'Actuator 3', 'Actuator 4', 'Actuator 5', 'Actuator 6', 'Min Length', 'Max Length', 'Location', 'northwest')
    title('Joint Velocities')
    hold off

    figure(3)
    plot(time, Px, time, Py, time, Pz)
    xlabel('Time (seconds)')
    ylabel('Coordinate Value (inches)')
    legend('X Centroid', 'Y Centroid', 'Z Centroid', 'Location', 'east')
    title('Position of Centroid P')

    figure(4)
    plot(time, theta, time, phi, time, psi)
    xlabel('Time (seconds)')
    ylabel('Angle of Rotation (degree)')
    legend('Theta (about X)', 'Phi (about Y)', 'Psi (about Z)')
    title('Angle of Rotation about Respective Axis')

    figure(5)
    plot(time_dot, Px_dot, time_dot, Py_dot, time_dot, Pz_dot, time_dot, P_dot)
    xlabel('Time (seconds)')
    ylabel('Coordinate Value Velocity (inches/sec)')
    legend('X Centroid', 'Y Centroid', 'Z Centroid', 'Centroid Velocity', 'Location', 'southeast')
    title('Velocity of Centroid P')

    figure(6)
    plot(time_dot, theta_dot, time_dot, phi_dot, time_dot, psi_dot)
    xlabel('Time (seconds)')
    ylabel('Angular Velocity (degree/sec)')
    legend('Theta Dot (about X)', 'Phi Dot (about Y)', 'Psi Dot (about Z)', 'Location', 'north')
    title('Angular Velocity about Respective Axis')
end

%% Animation
if animation
    %— 1) 한 번만 Figure 생성 & 초기화
    figure(100); 
    clf; 
    axis square; 
    grid on;
   
    
    %— 2) 베이스 원형 & 액추에이터 바닥 위치 계산
    alpha_base = 0:10:360;
    x_base = base_diameter/2 * cosd(alpha_base);
    y_base = base_diameter/2 * sind(alpha_base);
    z_base = zeros(size(x_base));
    
    if ~configuration_6_3 && ~configuration_6_6
        alpha = base_angle;
    else
        alpha = 0:60:360;
    end
    x = a * cosd(alpha);
    y = a * sind(alpha);
    z = zeros(size(x));
    
    %— 3) 시간 루프: cla로 축만 지우고 다시 그림
    for j = 1:length(psi)
        cla;  % 축 안의 그래픽만 제거
        
        % 베이스 원
        plot3(x_base, y_base, z_base, 'b-', 'LineWidth', 3);
        hold on;
        
        % 각 액추에이터 그리기
        for i = 1:6
            x_end = x(i) + L_vector(1,i,j);
            y_end = y(i) + L_vector(2,i,j);
            z_end = z(i) + L_vector(3,i,j);
            plot3([x(i) x_end], [y(i) y_end], [z(i) z_end], 'LineWidth', 3);
        end
        
        % 상판(Top plate) 다각형 완성
        Xd = [x + squeeze(L_vector(1,:,j)), x(1)+L_vector(1,1,j)];
        Yd = [y + squeeze(L_vector(2,:,j)), y(1)+L_vector(2,1,j)];
        Zd = [z + squeeze(L_vector(3,:,j)), z(1)+L_vector(3,1,j)];
        plot3(Xd, Yd, Zd, 'LineWidth', 3);
        
        % 축 설정 (한 번만 해도 되지만, 필요시 매 프레임 보정)
        axis([-(1.5*a) 1.5*a  -(1.5*a) 1.5*a  0  (max(Pz)+b/2)]);
        
        % 화면 즉시 갱신
        drawnow;
    end
end