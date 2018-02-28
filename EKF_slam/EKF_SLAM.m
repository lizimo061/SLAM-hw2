%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;

%==== TEST: Setup uncertianty parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;

%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====

% Write your code here...
k = 6;
landmark = zeros(length(measure),1);
landmark_cov = zeros(2*k);
for i = 1:k
    beta = measure(2*i-1);
    r = measure(2*i);
    landmark(2*i-1) = r*cos(beta);
    landmark(2*i) = r*sin(beta);
    Gtmp = [1 0 -r*sin(beta);
            0 1 r*cos(beta)];
    Qtmp = [-r*sin(beta) cos(beta);
            r*cos(beta)  sin(beta)];
    landmark_cov(2*i-1:2*i,2*i-1:2*i) = Gtmp*pose_cov*Gtmp' + Qtmp*measure_cov*Qtmp';
end
%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];
%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    % Write your code here...
   
    theta = last_x(3);
    x_pre = last_x;
    x_pre(1:3) = last_x(1:3) + [d*cos(theta);d*sin(theta);alpha];
    
    
    G = [1 0 -d*sin(theta);
         0 1 d*cos(theta);
         0 0 1];
     
    % Transform noise covariance
    T = [cos(theta) -sin(theta) 0;
         sin(theta) cos(theta)  0;
         0          0          1];
     
    F = eye(2*k+3);
    F(1:3,1:3) = G;
    N = zeros(2*k+3);
    N(1:3,1:3) = T*control_cov*T';
    P_pre = F*P*F' + N;
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    % Write your code here...
    for i = 1:6
        theta_pre = x_pre(3);
        beta = measure(2*i-1);
        r = measure(2*i);
        lx_pre = x_pre(2*i+2);
        ly_pre = x_pre(2*i+3);
        rx_pre = x_pre(1);
        ry_pre = x_pre(2);
        delta = [lx_pre-rx_pre;ly_pre-ry_pre];
        q = delta'*delta;
        z_pre = [wrapToPi(atan2(delta(2),delta(1))-theta_pre);sqrt(q)];
        
        F = zeros(5,2*k+3);
        F(1:3,1:3) = eye(3);
        F(4:5,2*i+2:2*i+3) = eye(2);
        H = [delta(2)          -delta(1)         -q  -delta(2)        delta(1);
            -sqrt(q)*delta(1) -sqrt(q)*delta(2)  0   sqrt(q)*delta(1) sqrt(q)*delta(2)];
        H = H/q;
        H = H*F;
        K = P_pre*H'/(H*P_pre*H' + measure_cov);
        x_pre = x_pre + K*([beta;r]-z_pre);
        P_pre = (eye(15)-K*H)*P_pre;
    end
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
l_gt = [3 6 3 12 7 8 7 14 11 6 11 12];
for i=1:6
    plot(l_gt(2*i-1),l_gt(2*i),'k*');
end

%==== Close data file ====
fclose(fid);
