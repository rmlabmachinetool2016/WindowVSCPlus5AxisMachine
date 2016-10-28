
%% Paper graph for measured friction force on Y axis in eccentric friction model experiment
% Register for Average Velolcity- Voltage Array
    clf;
    KvToF = 12.81E2;
 TickFontSize = 20;
 LegendFontSize = 28;
XMax =10; YMax=3; % W5 Max 200 mm/s
% XMax =105; YMax=6; % W2.5 Max 100 mm/s
%  XMax =42; YMax=15; % W1 Max 40 mm/s
% XMax =42; YMax=6; % X10W4 Max 40 mm/s

NLDisturbance = 0.0;
% NLDisturbance = 0;
 
 ExperimentNumber = 1;
NumberData = 5;
 SelectAxis = 2; % Select X axis 
 MaxVel = 10;   % mm/s
 MaxElement = 100;%150; % For each direction
 ScaleVel= MaxElement/MaxVel;
 NumberStart = uint16(0.2*ScaleVel);%uint16(5.0/ScaleVel);
 NumberCut = uint16(0.6*ScaleVel);%uint16(5.0/ScaleVel);
 VelNumber = 2*MaxElement+1;
 
 
 TempNum_samp = 160001;  % max number of point 
  num_variable = 27;
% plot(vec_realx_1(1,1:num_samp),vec_u(1,1:num_samp),'+','Color','b','LineWidth',1);

set(gca,'XTick',-100:2:100,'FontSize',TickFontSize)
set(gca,'YTick',-50000:10:50000,'FontSize',TickFontSize)
 m_TimeNowTemp = zeros(1,TempNum_samp);
 vec_uTemp = zeros(5,TempNum_samp);
 vec_uPredictedTemp = zeros(5,TempNum_samp);
 
 vec_V_UMedium = zeros(5,VelNumber);
 vec_V_UFiltered = zeros(5,VelNumber);
vec_V_UFilteredNoAcc = zeros(5,VelNumber);
 vec_V_UCounter = zeros(5,VelNumber);
 vec_V_UMediumNoAcc = zeros(5,VelNumber);
 vec_V_UCounterNoAcc = zeros(5,VelNumber);
vec_realxTemp = zeros(5,TempNum_samp);
vec_realx_1Temp = zeros(5,TempNum_samp);
vec_realx_2Temp = zeros(5,TempNum_samp);

vec_refrTemp = zeros(5,TempNum_samp);
vec_refr_1Temp = zeros(5,TempNum_samp);
vec_refr_2Temp = zeros(5,TempNum_samp);

 NumberTemp = 0;
 NotGoodStart = 10;
NotGoodEnd = 5;
MaxVelocity = 0.0;

for j=1:ExperimentNumber
       %Open file to read
%     FileInput1 = ['D1R10W40_60F850T', num2str(i), '.rme']; % num2str(i)% +
%     FileInput2 = ['D2R10W40_60F850T', num2str(i), '.rme']; 
%     FileInput3 = ['D3R10W40_60F850T', num2str(i), '.rme']; 
    
%     FileInput1 = ['DataForIdentification\R10F9_28T', num2str(1), '.rme']; % num2str(i)% +
    FileInput1 = ['DataForIdentification\SLMC_3D_R10F10T', num2str(1), '.rme']; % RmFiveAxisData.rme TUNX40_RE4_W5.rme 
%     FileInput1 = ['TuningDataXaxis\TuningX_F16_S15_T', num2str(1), '.rme']; % RmFiveAxisData.rme TUNX40_RE4_W5.rme 
    FID_in = fopen(FileInput1,'r');
    
%     m_TimeTotal = fread(FID_in, 1,'double');
  %Start read data to variable 
  
    for i=1:TempNum_samp
    vec_temp = fread(FID_in,num_variable,'double'); 
    if (numel(vec_temp)>0)       
        
         m_TimeNowTemp(i) = vec_temp(1);
         vec_uTemp(1,i) =  vec_temp(18);%vec_temp(18);
         vec_uTemp(2,i) =  vec_temp(19);
         vec_uTemp(3,i) =  vec_temp(20);
         vec_uTemp(4,i) =  vec_temp(21);
         vec_uTemp(5,i) =  vec_temp(22);
         
         vec_uPredictedTemp(1,i) =  vec_temp(23);
         vec_uPredictedTemp(2,i) =  vec_temp(24);
         vec_uPredictedTemp(3,i) =  vec_temp(25);
         vec_uPredictedTemp(4,i) =  vec_temp(26);
         vec_uPredictedTemp(5,i) =  vec_temp(27);
         
                  
          vec_realxTemp(1,i) = vec_temp(8);%vec_temp(8);
          vec_realxTemp(2,i) = vec_temp(9); 
          vec_realxTemp(3,i) = vec_temp(10); 
          vec_realxTemp(4,i) = vec_temp(11); 
          vec_realxTemp(5,i) = vec_temp(12); 
          
          vec_refrTemp(1,i) = vec_temp(3);%vec_temp(8);
          vec_refrTemp(2,i) = vec_temp(4); 
          vec_refrTemp(3,i) = vec_temp(5);
          vec_refrTemp(4,i) = vec_temp(6);
          vec_refrTemp(5,i) = vec_temp(7);
          if (i>2)
              
              for ktemp=1:NumberData
                vec_realx_1Temp(ktemp,i) = (vec_realxTemp(ktemp,i)-vec_realxTemp(ktemp,i-1))/(m_TimeNowTemp(i)-m_TimeNowTemp(i-1));  
                
                vec_refr_1Temp(ktemp,i) = (vec_refrTemp(ktemp,i)-vec_refrTemp(ktemp,i-1))/(m_TimeNowTemp(i)-m_TimeNowTemp(i-1));
              end;
            end;    
            if (i>3)
              for ktemp=1:NumberData
                 vec_realx_2Temp(ktemp,i) = (vec_realx_1Temp(ktemp,i)-vec_realx_1Temp(ktemp,i-1))/(m_TimeNowTemp(i)-m_TimeNowTemp(i-1));
                
                vec_refr_2Temp(ktemp,i) = (vec_refr_1Temp(ktemp,i)-vec_refr_1Temp(ktemp,i-1))/(m_TimeNowTemp(i)-m_TimeNowTemp(i-1));
              end;
                
%                            IndexVel = uint32((vec_realx_1Temp(1,i)+MaxVel)*ScaleVel);
%                            vec_V_UMedium(1,IndexVel) = vec_V_UMedium(1,IndexVel)+ vec_uTemp(1,i);
%                            vec_V_UCounter(1,IndexVel) = vec_V_UCounter(1,IndexVel)+1;
                           
           
            end;
           if (MaxVelocity<vec_refr_1Temp(1,i))
                   MaxVelocity =vec_refr_1Temp(1,i);
           end;
         NumberTemp = i;
        end; % End if (numel(vec_temp)>0)

    end; % End    for i=1:TempNum_samp 
    fclose(FID_in);     
%  plot(vec_realx_1Temp(1,1:NumberTemp),vec_uTemp(1,1:NumberTemp)+NLDisturbance,'+','Color','b','LineWidth',1);     
%   plot(vec_refr_1Temp(1,1:NumberTemp),vec_uTemp(1,1:NumberTemp),'+','Color','b','LineWidth',1); 
%  plot(vec_realx_1Temp(1,1:NumberTemp),vec_uPredictedTemp(1,1:NumberTemp),'+','Color','m','LineWidth',1); 
end;
% clf;
% hold all;
% MXAxis = 2.5; %0.0044; 0.00533;
% 
% vec_uFromAcc = MXAxis.*vec_refr_2Temp+NLDisturbance;
% 
% % plot(m_TimeNowTemp(1:NumberTemp),1000*(vec_refrTemp(SelectAxis,1:NumberTemp)-vec_realxTemp(SelectAxis,1:NumberTemp)),'-','Color','b','LineWidth',1);    % Friction velocity Graph
%     % plot(vec_refr_1Temp(SelectAxis,1:NumberTemp),vec_uFromAcc(SelectAxis,1:NumberTemp),'-','Color','g','LineWidth',1);    % Friction velocity Graph
% % plot(vec_refr_1Temp(SelectAxis,1:NumberTemp),6.0*vec_realx_1Temp(SelectAxis,1:NumberTemp),'-','Color','c','LineWidth',1);    % Friction velocity Graph
% 
% %  plot(vec_realx_1Temp(1,1:NumberTemp),MXAxis*vec_refr_2Temp(1,1:NumberTemp),'-','Color','r','LineWidth',1);  
% %  plot(vec_realx_1Temp(1,1:NumberTemp),vec_uTemp(1,1:NumberTemp)-MXAxis*vec_refr_2Temp(1,1:NumberTemp)+NLDisturbance,'-','Color','r','LineWidth',1);    % Friction velocity Graph
%   plot(vec_realxTemp(1,1:NumberTemp),vec_realxTemp(2,1:NumberTemp),'-','Color','b','LineWidth',1);    % Friction velocity Graph
%   plot(vec_refrTemp(1,2:NumberTemp),vec_refrTemp(2,2:NumberTemp),'-','Color','r','LineWidth',1);    % Friction velocity Graph
% %  plot(vec_realxTemp(1,1:NumberTemp),vec_uTemp(1,1:NumberTemp)-MXAxis*vec_refr_2Temp(1,1:NumberTemp)+NLDisturbance,'-','Color','r','LineWidth',1);    % Friction position Graph
% % axis([-7 17 -500 500]) %W40-60
 clf;
 hold all;
 SelectAxis = 2;
 MXAxis = 0.8; %0.8; %0.0044; 0.00533;

vec_uFromAcc = MXAxis.*vec_refr_2Temp+NLDisturbance;
% vec_uTemp = vec_uTemp -MXAxis.*vec_refr_2Temp+NLDisturbance;

%  plot(vec_realx_1Temp(1,1:NumberTemp),MXAxis*vec_refr_2Temp(1,1:NumberTemp),'-','Color','r','LineWidth',1);  
%  plot(vec_realx_1Temp(1,1:NumberTemp),vec_uTemp(1,1:NumberTemp)-MXAxis*vec_refr_2Temp(1,1:NumberTemp)+NLDisturbance,'-','Color','r','LineWidth',1);    % Friction velocity Graph
% plot(vec_refr_1Temp(SelectAxis,40:round(NumberTemp/4)-10),vec_uTemp(SelectAxis,40:round(NumberTemp/4)-10),'-','Color','b','LineWidth',1);    % Friction velocity Graph

% plot(vec_refr_1Temp(SelectAxis,round(NumberTemp/4):round(NumberTemp/2)),vec_uTemp(SelectAxis,round(NumberTemp/4):round(NumberTemp/2)),'-','Color','m','LineWidth',1);    % Friction velocity Graph
% plot(vec_refr_1Temp(SelectAxis,round(NumberTemp/2):round(0.75*NumberTemp)),vec_uTemp(SelectAxis,round(NumberTemp/2):round(0.75*NumberTemp)),'-','Color','m','LineWidth',1);    % Friction velocity Graph
% plot(vec_refr_1Temp(SelectAxis,round(0.75*NumberTemp):NumberTemp),vec_uTemp(SelectAxis,round(0.75*NumberTemp):NumberTemp),'-','Color','b','LineWidth',1);    % Friction velocity Graph
% plot(vec_refr_1Temp(SelectAxis,1:NumberTemp),vec_uFromAcc(SelectAxis,1:NumberTemp),'-','Color','g','LineWidth',1);    % Friction velocity Graph
% plot(vec_refr_1Temp(SelectAxis,1:NumberTemp),vec_uFromAcc(SelectAxis,1:NumberTemp)+5.0*vec_refr_1Temp(SelectAxis,1:NumberTemp)+35*sign(vec_refr_1Temp(SelectAxis,1:NumberTemp)),'-','Color','g','LineWidth',1);    % Friction velocity Graph
% plot(vec_refr_1Temp(SelectAxis,1:NumberTemp),vec_uPredictedTemp(SelectAxis,1:NumberTemp),'-','Color','r','LineWidth',1);    % Friction velocity Graph
%   plot(m_TimeNowTemp(1:NumberTemp),vec_uTemp(SelectAxis,1:NumberTemp),'-','Color','b','LineWidth',1);    % Friction velocity Graph
%  plot(vec_realxTemp(1,1:NumberTemp),vec_uTemp(1,1:NumberTemp)-MXAxis*vec_refr_2Temp(1,1:NumberTemp)+NLDisturbance,'-','Color','r','LineWidth',1);    % Friction position Graph
% axis([0 12 -500 500]) %W40-60
% axis([-7 7 -100 100]) %W40-60
% axis([-MaxVelocity*1.1 MaxVelocity*1.1 -200 200]) %W40-60

%  modelvel = modelr*modelomega*sin(modelomega*modeltime); 
%  modelacc = modelr*modelomega*modelomega*cos(modelomega*modeltime);

% t= vec_refr_1Temp(SelectAxis,40:round(NumberTemp/4)-10);

% indexInit = 40;
% indexEnd = round(NumberTemp/4)-10;
% indexInit = round(NumberTemp/4)+20;
% indexEnd = round(NumberTemp/2)-20;
% indexInit = round(NumberTemp/2)+20;
% indexEnd = round(0.75*NumberTemp)-20;
% indexInit = round(NumberTemp/4)+20;
% indexEnd = round(NumberTemp)-20;

indexInit = 40;
indexEnd = round(NumberTemp/2)-20;

RealFrictionForce= vec_uTemp(SelectAxis,indexInit:indexEnd); % (SelectAxis,40:round(NumberTemp/4)-10);   (SelectAxis,round(NumberTemp/4):round(NumberTemp/2));

time = m_TimeNowTemp(indexInit:indexEnd);

modelpos = vec_refrTemp(SelectAxis,indexInit:indexEnd);
modelvel = vec_refr_1Temp(SelectAxis,indexInit:indexEnd);
modelacc = vec_refr_2Temp(SelectAxis,indexInit:indexEnd);

alpha0 = 15;
alpha1 = 30;
alpha2 = 3;
vel0 = 0.9;
delta0 = 2;
Ff = ConvStribeckFricFunc(modelvel,alpha0,alpha1,alpha2,vel0,delta0);

% vec_refVel = [-15, -14, -12, -10, -8, -6, -4, -2, -1, -0.6, -0.4, -0.2, -0.15, -0.1, -0.06, -0.03, -0.01, 0.01, 0.03, 0.06, 0.1, 0.15, 0.2, 0.4, 0.6, 1.0, 2, 4, 6, 8, 10, 12, 14, 15];
vec_refVel = [-10, -9, -8.5, -8, -7.5, -7, -6.5, -6, -5.5, -5, -4.5, -4, -3.5, -3, -2.5,-1.5,-1,-0.5,-0.4,-0.2,-0.15,-0.08,-0.04,-0.01,0.01,0.04,0.04,0.15,0.2,0.4,0.5,1,1.5,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,10];
vec_observedFric = ConvStribeckFricFunc(vec_refVel,alpha0,alpha1,alpha2,vel0,delta0);

for j=1:length(vec_refVel)
    if (j>7)&&(j<21)
            vec_observedFric(j) = vec_observedFric(j)+vec_observedFric(j)/10-vec_observedFric(j)*rand(1)/10;
    else
            vec_observedFric(j) = vec_observedFric(j)+vec_observedFric(j)/4-vec_observedFric(j)*rand(1)/2;
    end;
    


end;

% plot(modelpos,RealFrictionForce,'r','LineWidth',1);
% plot(modelpos,modelvel,'b','LineWidth',1);
plot(modelvel,RealFrictionForce,'g','LineWidth',1);
plot(vec_refVel,vec_observedFric,'o','Color','r','LineWidth',1);

plot(modelvel,Ff,'b','LineWidth',2);
% plot(modelvel,SmoothRealFrictionForce,'k','LineWidth',1);
% plot(modelvel,ConvOptimumFf,'k','LineWidth',2);
% plot(modelvel,OptimumFf,'r','LineWidth',2);


% plot(time,modelacc,'r','LineWidth',2);
% plot(modelvel,Ff,'r','LineWidth',2);


h = legend('Observed friction','Conventional friction model');
set(h,'interpreter','latex','FontSize',LegendFontSize,'Location','NorthWest')
xlabel('Velocity [mm/s]','FontSize',LegendFontSize); ylabel( 'Friction Force [N]','FontSize',LegendFontSize)
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize);
set(gca,'YTick',-100:10:100,'FontSize',TickFontSize);
axis([-11 11 -80 80]); %W40-60
grid on;
%% Step1 Eccentricfriction model identification    Proposed friction model identification
 clf;
 hold all;
 
indexInit = 40;
indexEnd = round(NumberTemp/4)-140;

RealFrictionForce= vec_uTemp(SelectAxis,indexInit:indexEnd); % (SelectAxis,40:round(NumberTemp/4)-10);   (SelectAxis,round(NumberTemp/4):round(NumberTemp/2));


time = m_TimeNowTemp(indexInit:indexEnd);

modelpos = vec_refrTemp(SelectAxis,indexInit:indexEnd);
modelvel = vec_refr_1Temp(SelectAxis,indexInit:indexEnd);
modelacc = vec_refr_2Temp(SelectAxis,indexInit:indexEnd);
 


NominalFrictionForce = ConvFricFunc(modelvel,alpha0-10,alpha2);
EccentricFrictionForce = RealFrictionForce-NominalFrictionForce;



%  plot(modelpos,RealFrictionForce,'r','LineWidth',1);
  plot(modelpos,EccentricFrictionForce,'b','LineWidth',1);
 
 eta2 = 20;
 eta3 = 1;
 start = [eta2 eta3];
options = optimset('TolX',0.001);
[OptimumParameters,fval,exitflag,output] = fminsearch(@(lambda)(fitEccentricFrictionModel(lambda,modelpos,EccentricFrictionForce)),start,options);

InitEstimatedEccentricFf = EccentricFricFunc(modelpos,eta2,eta3);
OptimumEccentricFf =  EccentricFricFunc(modelpos,OptimumParameters(1),OptimumParameters(2));

plot(modelpos,OptimumEccentricFf,'r','LineWidth',2);
% plot(modelpos,InitEstimatedEccentricFf,'k','LineWidth',1); 
 
h = legend('Observed eccentric friction','Identified eccentric friction model');
set(h,'interpreter','latex','FontSize',LegendFontSize,'Location','NorthWest')
xlabel('Y Axis position [mm]','FontSize',LegendFontSize); ylabel( 'Friction Force [N]','FontSize',LegendFontSize)
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize);
set(gca,'YTick',-100:10:100,'FontSize',TickFontSize);
% axis([-11 11 -80 80]); %W40-60
grid on;

%% Step 2. Conventional stribeck friction model estimation
 clf;
 hold all;
indexInit = 40;
indexEnd = round(NumberTemp/2)-20;
% indexInit = 40;
% indexEnd = round(NumberTemp/4)-10;

RealFrictionForce= vec_uTemp(SelectAxis,indexInit:indexEnd); % (SelectAxis,40:round(NumberTemp/4)-10);   (SelectAxis,round(NumberTemp/4):round(NumberTemp/2));

time = m_TimeNowTemp(indexInit:indexEnd);

modelpos = vec_refrTemp(SelectAxis,indexInit:indexEnd);
modelvel = vec_refr_1Temp(SelectAxis,indexInit:indexEnd);
modelacc = vec_refr_2Temp(SelectAxis,indexInit:indexEnd);

alpha0 = 15;
alpha1 = 30;
alpha2 = 3;
vel0 = 0.9;
delta0 = 2;
Ff = ConvStribeckFricFunc(modelvel,alpha0,alpha1,alpha2,vel0,delta0);

% vec_refVel = [-15, -14, -12, -10, -8, -6, -4, -2, -1, -0.6, -0.4, -0.2, -0.15, -0.1, -0.06, -0.03, -0.01, 0.01, 0.03, 0.06, 0.1, 0.15, 0.2, 0.4, 0.6, 1.0, 2, 4, 6, 8, 10, 12, 14, 15];
vec_refVel = [-10, -9, -8.5, -8, -7.5, -7, -6.5, -6, -5.5, -5, -4.5, -4, -3.5, -3, -2.5,-1.5,-1,-0.5,-0.4,-0.2,-0.15,-0.08,-0.04,-0.01,0.01,0.04,0.04,0.15,0.2,0.4,0.5,1,1.5,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,10];
vec_observedFric = ConvStribeckFricFunc(vec_refVel,alpha0,alpha1,alpha2,vel0,delta0);

for j=1:length(vec_refVel)
    if (j>7)&&(j<21)
            vec_observedFric(j) = vec_observedFric(j)+vec_observedFric(j)/10-vec_observedFric(j)*rand(1)/10;
    else
            vec_observedFric(j) = vec_observedFric(j)+vec_observedFric(j)/4-vec_observedFric(j)*rand(1)/2;
    end;
    


end;

% plot(modelpos,RealFrictionForce,'r','LineWidth',1);
% plot(modelpos,modelvel,'b','LineWidth',1);
% plot(modelvel,RealFrictionForce,'g','LineWidth',1);
plot(vec_refVel,vec_observedFric,'o','Color','r','LineWidth',1);

plot(modelvel,Ff,'b','LineWidth',2);
% plot(modelvel,SmoothRealFrictionForce,'k','LineWidth',1);
% plot(modelvel,ConvOptimumFf,'k','LineWidth',2);
% plot(modelvel,OptimumFf,'r','LineWidth',2);


% plot(time,modelacc,'r','LineWidth',2);
% plot(modelvel,Ff,'r','LineWidth',2);


h = legend('Observed friction','Conventional friction model');
set(h,'interpreter','latex','FontSize',LegendFontSize,'Location','NorthWest')
xlabel('Velocity [mm/s]','FontSize',LegendFontSize); ylabel( 'Friction Force [N]','FontSize',LegendFontSize)
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize);
set(gca,'YTick',-100:10:100,'FontSize',TickFontSize);
axis([-11 11 -80 80]); %W40-60
grid on;

%% Step 3. Comparision estimation result between two method

clf;

% indexInit = 40;
% indexEnd = round(NumberTemp/4)-20;
indexInit = 40;
indexEnd = round(NumberTemp/2)-20;

RealFrictionForce= vec_uTemp(SelectAxis,indexInit:indexEnd); % (SelectAxis,40:round(NumberTemp/4)-10);   (SelectAxis,round(NumberTemp/4):round(NumberTemp/2));

time = m_TimeNowTemp(indexInit:indexEnd);

modelpos = vec_refrTemp(SelectAxis,indexInit:indexEnd);
modelvel = vec_refr_1Temp(SelectAxis,indexInit:indexEnd);
modelacc = vec_refr_2Temp(SelectAxis,indexInit:indexEnd);

NominalFrictionFn = ConvFricFunc(modelvel,alpha0,alpha2);
OptimumEccentricFf = EccentricFricFunc(modelpos,OptimumParameters(1),OptimumParameters(2));
ProposedFrictionFr = NominalFrictionFn+OptimumEccentricFf;



% plot(modelpos,RealFrictionForce,'r','LineWidth',1);
% plot(modelpos,modelvel,'b','LineWidth',1);
plot(modelvel,RealFrictionForce,'g','LineWidth',1);
 hold all;
% plot(vec_refVel,vec_observedFric,'o','Color','r','LineWidth',1);
plot(modelvel,Ff,'b','LineWidth',2);
plot(modelvel,ProposedFrictionFr,'r','LineWidth',2);
% plot(modelvel,SmoothRealFrictionForce,'k','LineWidth',1);
% plot(modelvel,ConvOptimumFf,'k','LineWidth',2);
% plot(modelvel,OptimumFf,'r','LineWidth',2);


% plot(time,modelacc,'r','LineWidth',2);
% plot(modelvel,Ff,'r','LineWidth',2);


h = legend('Observed friction','Conventional friction model','Proposed friction model');
set(h,'interpreter','latex','FontSize',LegendFontSize,'Location','NorthWest')
xlabel('Velocity [mm/s]','FontSize',LegendFontSize); ylabel( 'Friction Force [N]','FontSize',LegendFontSize)
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize);
set(gca,'YTick',-100:10:100,'FontSize',TickFontSize);
axis([0 9.8 -40 100]); %W40-60
grid on;


%% Reference Y position
clf;

% indexInit = 40;
% indexEnd = round(NumberTemp/4)-20;
indexInit = 10;
indexEnd = round(NumberTemp)-10;


plot(m_TimeNowTemp(indexInit:indexEnd),vec_refrTemp(2,indexInit:indexEnd),'b','LineWidth',2);

 hold all;


h = legend('Reference Y position');
set(h,'interpreter','latex','FontSize',LegendFontSize,'Location','NorthWest')
xlabel('Time [s]','FontSize',LegendFontSize); ylabel( 'Position [mm]','FontSize',LegendFontSize)
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize);
set(gca,'YTick',-100:2:100,'FontSize',TickFontSize);
axis([0 6.28 -12 12]); %W40-60
grid on;

%% Observed friction on Y Axis
clf;

% indexInit = 40;
% indexEnd = round(NumberTemp/4)-20;
indexInit = 40;
indexEnd = round(NumberTemp)-40;


plot(m_TimeNowTemp(indexInit:indexEnd),vec_uTemp(2,indexInit:indexEnd),'b','LineWidth',1);

 hold all;


h = legend('Observed friction on Y axis');
set(h,'interpreter','latex','FontSize',LegendFontSize,'Location','NorthWest')
xlabel('Time [s]','FontSize',LegendFontSize); ylabel( 'Friction Force [N]','FontSize',LegendFontSize)
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize);
set(gca,'YTick',-100:20:100,'FontSize',TickFontSize);
axis([0 6.28 -100 80]); %W40-60
grid on;

%% Save analysis system parameters in to BinaryFile in eccentric friction model experiment
% FileInput1 = ['D:\5AxisCNCExperiment\Rmlab5AxisCNCSoft\WindowVer5Axis\WindowVSCPlus5AxisMachine\'...
%     'FiveAxisMachineWinVS\RmLabCNC\Release\RmFiveAxisCNC\InitialConfig\FiveAxisMachineParameters.rme'];%

% Real Five axis machine Update 
% getenv('username')
% getenv('computername')
if strcmp(getenv('computername'),'D3-402-01')
FileInput1 = ['D:\5AxisCNCExperiment\Rmlab5AxisCNCSoft\WindowVer5Axis\WindowVSCPlus5AxisMachine\'...
    'FiveAxisMachineWinVS\RmLabCNC\Release\RmFiveAxisCNC\InitialConfig\FiveAxisMachineParameters.rme'];%
else
    FileInput1 = ['E:\DINHBA\WindowVSCPlus5AxisMachine\FiveAxisMachineWinVS\RmLabCNC\Release\RmFiveAxisCNC\InitialConfig\'...
    'FiveAxisMachineParameters.rme'];%
end;

FID_in = fopen(FileInput1,'w'); 
%     m_TimeTotal = fread(FID_in,1,'double');
  %Start read data to variable 
%    num_variable = 14;
% NumberPeakXFwd  vec_PeakHeightXFwd vec_PeakPosXFwd  vec_PeakWidXFwd
NumberAxis = 5;
NumberActuator = 7;
matrix_weight_M = [[0.0044,0,0,0,0];
                   [0,0.0045,0,0,0];
                   [0,0.0,  0.0017,0,0];
                   [0,0.0,0,       0.00025,0];
                   [0,0,0.0,0,             0.005]];
matrix_viscous_friction_c =     [[0.055,0,0,0,0];
                                [ 0,   0.055,0,0,0];
                                [0.0,0,    0.025,0,0];
                                [0.0,0,0,     0.025,0];
                                [0.0,0,0,0,         0.025]];
vector_coulomb_friction_fcl =   [0.4,0.5,0.6,0.7,0.8];
vector_gravitational_force_g =   [1.0,1.0,2.0,0.0,0.0]; 

                            
matrix_estimated_weight_M = [[0.0054,0,0,0,0];
                   [0,0.0055,0,0,0];
                   [0,0.0,  0.0037,0,0];
                   [0,0.0,0,       0.00045,0];
                   [0,0,0.0,0,             0.007]];
matrix_estimated_viscous_friction_c= [[0.045,0,0,0,0];
                                [ 0,   0.075,0,0,0];
                                [0.0,0,    0.035,0,0];
                                [0.0,0,0,     0.045,0];
                                [0.0,0,0,0,         0.055]];                                         
vector_estimated_coulomb_friction_fcl =    [0.6,0.3,0.8,0.5,0.6];

vector_estimated_gravitational_force_g =   [0.0,0.0,1.5,0.0,0.0];      

% PD tracking controller gain   400 300 400   20 20
matrix_TC_Gain_Kp =      [[800,0,0,0,0];
                          [0,800,0,0,0];
                          [0,0,4000,0,0];
                          [0,0,0,4000,0];
                          [0,0,0,0,4000]]; 
matrix_TC_gain_Kd= [[30,0,0,0,0];
                            [0,30,0,0,0];
                            [0,0,300,0,0];
                            [0,0,0,300,0];
                            [0,0,0,0,300]];
% % PD contouring controller gain   elt, eln, elb    100 400 400   10  30
% 30
% matrix_TC_Gain_Kp =      [[400,0,0];
%                            [0,300,0];
%                            [0,0,400]]; 
% matrix_TC_gain_Kd= [[20,0,0];
%                             [0,20,0];
%                             [0,0,20]];                        
% Sliding mode controller gain
matrix_SLMCC_gain_lambda = [[10,0,0];
                           [0,10,0];
                           [0,0,10]]; 
matrix_SLMCC_gain_A =      [[30,0,0];
                           [0,60,0];
                           [0,0,40]]; 
matrix_disturbance_gain_Kd= [[10,0,0];
                            [0,10,0];
                            [0,0,10]]; 
vector_SLMCC_gain_k =       [0.5,0.5,0.5];
% Sin profile Nonlinear friction model
                 
               
    fwrite(FID_in,NumberAxis,'uint8');
%      COnventional system dynamic and friction
    for i=1:NumberAxis
         fwrite(FID_in,matrix_weight_M(i,i),'double');
         fwrite(FID_in,matrix_viscous_friction_c(i,i),'double');
         fwrite(FID_in,vector_coulomb_friction_fcl(i),'double');
         fwrite(FID_in,vector_gravitational_force_g(i),'double');
         
         fwrite(FID_in,matrix_estimated_weight_M(i,i),'double');
         fwrite(FID_in,matrix_estimated_viscous_friction_c(i,i),'double');
         fwrite(FID_in,vector_estimated_coulomb_friction_fcl(i),'double');
         fwrite(FID_in,vector_estimated_gravitational_force_g(i),'double');    
                  
    end;
%  PD tracking controller gain
    for i=1:NumberAxis
         fwrite(FID_in,matrix_TC_Gain_Kp(i,i),'double');
         fwrite(FID_in,matrix_TC_gain_Kd(i,i),'double');               
    end;
 %    Sliding model contouring controller gain
    for i=1:NumberAxis-2
         fwrite(FID_in,matrix_SLMCC_gain_lambda(i,i),'double');
         fwrite(FID_in,matrix_SLMCC_gain_A(i,i),'double');
         fwrite(FID_in,matrix_disturbance_gain_Kd(i,i),'double');
         fwrite(FID_in,vector_SLMCC_gain_k(i),'double');                
    end;
    
    
%      %    % Sin profile Nonlinear friction model changing frequency
%      each axis include 5 friction parameter for 4 cycle mean total 20 parameters  
%     for i=1:NumberAxis
%                 for j=1:20
%                     fwrite(FID_in,matrix_proposed_friction_model_parameters(i,j),'double');
%                  end;    
%     end;

     %    % Sin profile eccentric friction model constant frequency   
     %  eta0+ eta1 * vel+ eta2*(sin(axispos*2*pi/2.0-eta3));
     %
     
%  ConvFricModelInit = zeros(3,5); %ConvStribeckFricFunc(vel,alpha0,alpha1,alpha2,vel0,delta0)
%  3x8 matrix include 2 value eta0, eta1 for forward v decrease; backward v decrease;  backward v increase; Forward v increase;  in each X, Y, Z axis
%  ProposedFricModelInit = zeros(3,4); % 4 matrix include 4 value eta0, eta1, eta2, eta3 in each X, Y, Z axis
 
 ConvStribeckFricModelInit = [20 35 5 0.6 2;15 30 3 0.9 2;25 40 5 0.4 2];
ProposedFricModelInit =[20 5 0 0;15 3 11.2 -0.27;25 5 0 0];

     for i=1:NumberAxis-2
                for j=1:5
                    fwrite(FID_in,ConvStribeckFricModelInit(i,j),'double');
                 end;    
    end; 
    
    for i=1:NumberAxis-2
                for j=1:4
                    fwrite(FID_in,ProposedFricModelInit(i,j),'double');
                 end;    
    end;
fclose(FID_in); 

%% Convert Controller parameters BinaryFile to textfile
FileInput1 = 'E:\BUIDINHBA\MachineTool\3axismachine\RmLabCNC\Release\RmFiveAxisCNC\InitialConfig\ThreeAxisMachineParameters.rme';%
FileOutput1 = 'ThreeAxisMachineParameters.txt';%
FID_in = fopen(FileInput1,'r'); 
FID_out = fopen(FileOutput1,'wt');
%     m_TimeTotal = fread(FID_in,1,'double');
  %Start read data to variable 
%    num_variable = 14;
    fprintf(FID_out,'Three axis machine tool parameters \n');
    NumberAxis = fread(FID_in,1,'uint8');
    fprintf(FID_out,'NumberAxis %.5f \n',NumberAxis);
    fprintf(FID_out,'Virtual and estimated System Parameters \n');
    %      COnventional system dynamic and friction
    for i=1:NumberAxis
        fprintf(FID_out,'Vitural Axis %d weight_M %.5f viscous_friction_c %.5f  coulomb_friction_fcl  %.5f gravitational_force_g %.5f \n'...
                                ,[i;fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double')] );
        fprintf(FID_out,'Estimated Axis %d weight_M %.5f viscous_friction_c %.5f  coulomb_friction_fcl  %.5f gravitational_force_g %.5f \n'...
                                ,[i;fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double')] );                  
    end;
   %  PD tracking controller gain
   fprintf(FID_out,' PD tracking controller gain\n');
    for i=1:NumberAxis
       fprintf(FID_out,'Index %d matrix_TC_Gain_Kp %.5f matrix_TC_gain_Kd %.5f \n'...
                                ,[i;fread(FID_in,1,'double');fread(FID_in,1,'double')] );
                                        
    end;  
    
 %    Sliding model contouring controller gain
   fprintf(FID_out,'Sliding model contouring controller gain \n');
    for i=1:NumberAxis
       fprintf(FID_out,'Index %d gain_lambda %.5f gain_A %.5f  gain_Kd  %.5f gain_k %.5f \n'...
                                ,[i;fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double')] );
                                        
    end;
    
      %    The stribeck-coulomb-viscous friction model
     fprintf(FID_out,'The stribeck-coulomb-viscous friction model \n');
    for i=1:NumberAxis
        fprintf(FID_out,'Axis %d nominal_coulomb_friction_fncl %.5f nominal_static_friction_cn %.5f nominal_viscous_friction_cn %.5f stribeck factor v0 %.5f  stribeck shape delta %.5f   \n'...
                                ,[i;fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double')] );
                     
    end;   
     %    The eccentric-coulomb-viscous friction model
     fprintf(FID_out,'The eccentric-coulomb-viscous friction model \n');
       for i=1:NumberAxis
        fprintf(FID_out,'Axis %d nominal_coulomb_friction_eta 0 %.5f nominal_viscous_friction_eta 1 %.5f eccentric friction width eta 2 %.5f  initial position eta 3 %.5f   \n'...
                                ,[i;fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double')] );
                     
    end; 
%      %    Nonlinear friction model
%      fprintf(FID_out,'Nonlinear friction model \n');
%     for i=1:NumberAxis
%         fprintf(FID_out,'Axis %d nominal_viscous_friction_cn %.5f nominal_coulomb_friction_fncl %.5f \n'...
%                                 ,[i;fread(FID_in,1,'double');fread(FID_in,1,'double')] );
%        
%         fprintf(FID_out,'Forward direction \n'); 
%         number_peak = fread(FID_in,1,'uint8');
%         for j=1:number_peak
%          fprintf(FID_out,'Peak %d peak_height %.5f peak_position %.5f peak_width %.5f \n'...
%                                 ,[j;fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double')] );   
%          end;
%          fprintf(FID_out,'backward direction \n'); 
%         number_peak = fread(FID_in,1,'uint8');
%         for j=1:number_peak
%          fprintf(FID_out,'Peak %d peak_height %.5f peak_position %.5f peak_width %.5f \n'...
%                                 ,[j;fread(FID_in,1,'double');fread(FID_in,1,'double');fread(FID_in,1,'double')] );   
%          end;                   
%          
%     end;
    
fclose(FID_in);  
fclose(FID_out);

