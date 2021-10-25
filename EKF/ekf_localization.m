


function [] = ekf_localization()
 

    close all;
   
    clear all;

    disp('EKF Start!')

    time = 0;
    global endTime; % [sec]
    endTime = 60;
    global dt;
    dt = 0.1; % [sec]

    removeStep = 5;

    nSteps = ceil((endTime - time)/dt);

    estimation.time=[];
    estimation.u=[];
    estimation.GPS=[];
    estimation.xOdom=[];
    estimation.xEkf=[];
    estimation.xTruth=[];

    % State Vector [x y yaw]'
    xEkf=[0 0 0]';
    PxEkf = eye(3);

    % Ground True State
    xTruth=xEkf;

    % Odometry Only
    xOdom=xTruth;

    % Observation vector [x y yaw]'
    z=[0 0 0]';

    % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.5 0.5 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
     convQ=noiseQ.^2;

    % Covariance Matrix for observation
     convR=noiseR.^2;

    % Other Intial
     xPred =[0;0;0];

    % Main loop
    for i=1 : nSteps
        time = time + dt;
        % Input
        u=robotControl(time);
        % Observation
        [z,xTruth,xOdom,u]=prepare(xTruth, xOdom, u);

        % ------ Kalman Filter --------
        % Predict
        F = jacobF(xEkf, u);
        xPred =  doMotion(xEkf, u);      %���˶�ģ�͵õ��ľ�ֵ
        PxEkf = F * PxEkf * F' + convQ;  %���·���
        
        % Update
        H = jacobH(xPred);                      %���ſ˱Ⱦ���H
        K = PxEkf*H'*(H*PxEkf*H'+convR)^-1; %��۲�����е�Kt
        xEkf= doObservation(z,xPred,K);     %���¾�ֵ
        PxEkf=(eye(3)-K*H)*PxEkf;           %���·������
        % -----------------------------
             
        

        % Simulation estimation
        estimation.time=[estimation.time; time];
        estimation.xTruth=[estimation.xTruth; xTruth'];
        estimation.xOdom=[estimation.xOdom; xOdom'];
        estimation.xEkf=[estimation.xEkf;xEkf'];
        estimation.GPS=[estimation.GPS; z'];
        estimation.u=[estimation.u; u'];

        % Plot in real time
        % Animation (remove some flames)
        if rem(i,removeStep)==0
            %hold off;
            plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
            plot(estimation.xOdom(:,1),estimation.xOdom(:,2),'.k', 'MarkerSize', 10);hold on;
            plot(estimation.xEkf(:,1),estimation.xEkf(:,2),'.r','MarkerSize', 10);hold on;
            plot(estimation.xTruth(:,1),estimation.xTruth(:,2),'.b', 'MarkerSize', 10);hold on;
            axis equal;
            grid on;
            drawnow;
            %movcount=movcount+1;
            %mov(movcount) = getframe(gcf);
        end 
    end
    close
    
    finalPlot(estimation);
 
end

% control
function u = robotControl(time)
    global endTime;

    T = 10; % sec
    Vx = 1.0; % m/s
    Vy = 0.2; % m/s
    yawrate = 5; % deg/s
    
    % half
    if time > (endTime/2)
        yawrate = -5;
    end
    
    u =[ Vx*(1-exp(-time/T)) Vy*(1-exp(-time/T)) degreeToRadian(yawrate)*(1-exp(-time/T))]';
    
end

% all observations for 
function [z, xTruth, xOdom, u] = prepare(xTruth, xOdom, u)
    global noiseQ;
    global noiseR;

    % Ground Truth
    xTruth=doMotion(xTruth, u);
    % add Motion Noises
    u=u+noiseQ*randn(3,1);
    % Odometry Only
    xOdom=doMotion(xOdom, u);
    % add Observation Noises
    z=xTruth+noiseR*randn(3,1);
end


% Motion Model
function x = doMotion(x, u)
    global dt;
    v = sqrt(u(1)^2+u(2)^2);
    trans = v * dt;
    a = trans * cos(x(3)+u(3));
    b = trans * sin(x(3)+u(3));
    u_trans = [a;b;u(3)*dt];
    x = x + u_trans;
end

% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    v = sqrt(u(1)^2+u(2)^2);
    trans = v * dt;
    a = -trans * sin(x(3)+u(3));
    b = trans * cos(x(3)+u(3));    
    jF=[1 0 a
        0 1 b
        0 0 1];
end

%Observation Model
function x = doObservation(z,xPred,K)
    x = xPred + K * (z-xPred);
    
 end

%Jacobian of Observation Model
function jH = jacobH(x)
   jH = eye(3);
end

% finally plot the results
function []=finalPlot(estimation)
    figure;
    nSteps = 600;
    plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
    plot(estimation.xOdom(:,1), estimation.xOdom(:,2),'.k','MarkerSize', 10); hold on;
    plot(estimation.xEkf(:,1), estimation.xEkf(:,2),'.r','MarkerSize', 10); hold on;
    plot(estimation.xTruth(:,1), estimation.xTruth(:,2),'.b','MarkerSize', 10); hold on;
    legend('GPS Observations','Odometry Only','EKF Localization', 'Ground Truth');

    xlabel('X (meter)', 'fontsize', 12);
    ylabel('Y (meter)', 'fontsize', 12);
    grid on;
    axis equal;
    
    % calculate error
     error_Odom = 0;
     error_EKF = 0; 
    for i=1:nSteps
        error_Odom = error_Odom + sqrt((estimation.xOdom(i,1)-estimation.xTruth(i,1))^2 +(estimation.xOdom(i,2)-estimation.xTruth(i,2))^2); 
        error_EKF = error_EKF + sqrt((estimation.xEkf(i,1)-estimation.xTruth(i,1))^2 +(estimation.xEkf(i,2)-estimation.xTruth(i,2))^2);    
    end
    %����ƽ�����
    error_Odom = error_Odom/nSteps
    error_EKF = error_EKF/nSteps
end

function radian = degreeToRadian(degree)
    radian = degree/180*pi;
end