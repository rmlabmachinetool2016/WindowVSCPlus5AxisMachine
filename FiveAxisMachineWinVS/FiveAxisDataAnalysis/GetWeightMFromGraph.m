%% Read data
%EXPERIMENTGRAPH Summary of this function goes here
    %   Detailed explanation goes here
    clc
    clear all;
    FileInput = 'Y_0_5V_To_5V_Range0_1Time0_05.rme';%'R5Time10W0.rme';%'R5Time10W0KcChange.rme';%'R5Time10W0.rme';
    disp('Start Get Data from VelocityGraph File '); 
    disp(FileInput);
    TempNum_samp = 2001;
    num_samp = TempNum_samp;
    m_TimeTotal = 0.0;
    m_SampTime = 0.05;
    m_TimeNow = zeros(1,num_samp);
	vec_realx_1 = zeros(2,num_samp);
    vec_realx_2 = zeros(2,num_samp);
	vec_u = zeros(1,num_samp);
    num_variable = 4;
    vec_temp = zeros(1,num_variable);
    %Open file to read
    FID_in = fopen(FileInput,'r'); 
    m_TimeTotal = fread(FID_in,1,'double');
    
  %Start read data to variable 
    for i=1:TempNum_samp
    vec_temp = fread(FID_in,num_variable,'double'); 
    if (numel(vec_temp)>0)
        m_TimeNow(i) = vec_temp(1);vec_u(i) = vec_temp(2);
        vec_realx_1(1,i) =  vec_temp(3);vec_realx_1(2,i) =  vec_temp(4);
        if (i>=6)
        vec_realx_2(1,i) =  (vec_realx_1(1,i)-vec_realx_1(1,i-5))/(5*m_SampTime);
        vec_realx_2(2,i) =  (vec_realx_1(2,i)-vec_realx_1(2,i-5))/(5*m_SampTime);
        end;
%         vec_realx_2(2,i) = vec_realx_2(1,i);% With X axis
%         vec_realx_1(2,i) = vec_realx_1(1,i);
      elseif (num_samp==TempNum_samp)
            num_samp = i;
        end;
    end;
    num_samp= num_samp-1;
    
    fclose(FID_in);   
%% Graph Plot
%Draw Vol _velocity Graph
clf;
plot(m_TimeNow(1,2:80),vec_realx_1(2,2:80),'-','Color','b');
% plot(m_TimeNow(1,:),vec_realx(1,:));
set(gca,'XTick',0:1:5)
set(gca,'YTick',0:1:10)
legend('Contour','Real Y');
axis([-1 5 0 11]);
xlabel('Vol'); ylabel('mm/s');
%% Velocity Viscous Graph
% plot(vec_refr_1Coarse(1,1:25),vec_ViscousCoarse(1,1:25),'-','Color','g');
clf;
set(gca,'XTick',0:0.05:11)
set(gca,'YTick',0:0.5:30)
% set(gca, 'ColorOrder', jet(3));
hold all;
% plot(vec_refr_1Coarse(1,26:50),vec_ViscousCoarse(1,26:50),'-','Color','c');
% plot(vec_refr_1Coarse(1,51:75),vec_ViscousCoarse(1,51:75),'-','Color','b');
% plot(vec_refr_1Coarse(1,76:end),vec_ViscousCoarse(1,76:end),'-','Color','m');
% plot(m_TimeNowCoarse(1,:),vec_uCoarse(2,:),'-','Color','c');
% plot(m_TimeNowCoarse(1,:),vec_ViscousCoarse(1,:),'-','Color','b');
% plot(m_TimeNow(1,:),vec_refr_1F2(1,:),'-','Color','b');
% plot(m_TimeNow(1,:),vec_refr_1F2(2,:),'-','Color','m');

% x = [0,0.3,1.2,2,4,5.2,8.6,9.2,10.2,10.4]; % simple set of x values
% y = [5,3.33,1.25,0.8,0.45,0.38,0.25,0.27,0.29,0.48]; % random y values for each x value
%R14Time10W0KcConstKv30Kp60.rme
% x = [0, 0.25,0.305 ,0.45 ,0.57 ,0.8 ,1.2 ,2   ,4   ,5.2 ,8.6 ,9.2 ,9.97  ,10.2 ,10.4 ,10.5]; % simple set of x values
% y = [20,3.05,2.79  ,2.31 ,2.02 ,1.59,1.06,0.65,0.40,0.35,0.25,0.28,0.335 ,0.45 ,0.85 ,1.2]; % random y values for each x value
%R14Time10W0KcConstKv30Kp60.rme More Better
x = [0, 0.25,0.305 ,0.45 ,0.57 ,0.8 ,1.2 ,2   ,4   ,5.2 ,8.6 ,9.2 ,9.97  ,10.2 ,10.4 ,10.5]; % simple set of x values
y = [20,3.05,2.70  ,2.21 ,1.92 ,1.45,1.02,0.65,0.40,0.35,0.25,0.28,0.335 ,0.45 ,0.85 ,1.2]; % random y values for each x value
%Default
% x = [0,0.3 ,1.2 ,2  ,4   ,5.2 ,8.6 ,9.2 ,10.2,10.4,10.5]; % simple set of x values
% y = [5,3.33,1.25,0.8,0.45,0.35,0.25,0.28,0.45,0.55,0.7]; % random y values for each x value
xx = linspace(0,10.5,1000); % many x values for plotting
yy = spline(x,y,xx); % generate y values based on spline interpolation
vec_realx_1Simple= vec_realx_1(2,2:num_samp);
vec_uSimple= vec_u(1,2:num_samp);
vec_Kc= spline(x,y,vec_realx_1Simple);
vec_uNonAcc = vec_Kc.*vec_realx_1Simple;
vec_uMxa= vec_uSimple- vec_uNonAcc;
% plot(m_TimeNow(1,2:80),vec_uMxa,'-','Color','b');
plot(m_TimeNow(1,2:num_samp),vec_realx_1(2,2:num_samp),'-','LineWidth',2,'Color','c');
plot(m_TimeNow(1,2:num_samp),vec_realx_2(2,2:num_samp),'-','LineWidth',2,'Color','g');
vec_MEper= vec_uMxa./vec_realx_2(2,2:num_samp);
vec_MEperFit= vec_MEper(1,7:22);
plot(m_TimeNow(1,7:22),vec_MEperFit,'-','LineWidth',2,'Color','m');
MeanWeight = mean(vec_MEperFit);
spl=spline(x,y);
% disp(spl.coefs);
% plot(x,y,'ro'); % red data points
% hold on;
% plot(xx,yy,'-','Color','r'); % Red spline curve

legend('Exp Velocity','Exp Acceleration','Mean weight','vec_MEper', 'RealCurve');
xlabel('Time second'); ylabel('Exp Velocity, Acceleration, Mean weight')
%axis([0.35 1.01 0 0.08])
axis([0.35 1.01 0 14])